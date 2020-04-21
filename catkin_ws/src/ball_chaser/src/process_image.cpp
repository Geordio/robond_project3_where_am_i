#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
// #include <string>

// Define a global client that can request services
ros::ServiceClient client;

// enum Location { Unknown, Left, Middle, Right };
const float MAXANG = 0.5;
const float MAXLIN = 0.5;
const float STRAIGHTANG = 0.0;
const float STOPPEDLIN = 0.0;

bool DEBUG = false;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  // ROS_INFO_STREAM("Driving the robot");

  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
  ROS_ERROR("Failed to call service");
}


// determine the required robot command based on the ball location
std::tuple<float, float> determine_command(int ballLocation) {

  int leftCentreThreshold = 250;
  int rightCentreThreshold = 550;
  float lin_x;
  float ang_z;

  if ((ballLocation < leftCentreThreshold) && (ballLocation >= 0)) {
    // command left
    if (DEBUG){
      ROS_INFO("go left");
    }
    lin_x = MAXLIN;
    ang_z = MAXANG;
  }
  else if ((ballLocation >= leftCentreThreshold) && (ballLocation <= rightCentreThreshold)) {
    // command left
    if (DEBUG) {
      ROS_INFO("go straight");
    }
    lin_x = MAXLIN;
    ang_z = STRAIGHTANG;
    // srv
  }
  else if  (ballLocation > rightCentreThreshold) {
    // command left
    if (DEBUG) {
      ROS_INFO("go right");
    }
    lin_x = MAXLIN;
    ang_z = -MAXANG;
  }
  else {
    // commaned stop
    if (DEBUG) {
      ROS_INFO("stop");
    }
    lin_x = STRAIGHTANG;
    ang_z = STOPPEDLIN;
  }
  return std::make_tuple (lin_x, ang_z);
}

// This callback function continuously executes and reads the image data
// basic process:
// iterate through the data array of the image
//    1. find the first white pixel (which will be left of first row of ball)
//    2. interate through remaining pixels in image row to find the last white pixel on row
//    3. find the centre of the white pixels in the row.
//    4. pass the x value of this pixel to method to determine if in left, centre or right
//    5. call method that can call the service,
void process_image_callback(const sensor_msgs::Image img)
{

  int white_pixel = 255;
  int img_height = img.height;
  int img_width = img.width;
  int img_data_array_size = img.data.size();
  int img_num_pixels = img_height * img_width;
  // Location location = Unknown;

  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;
  // std::string msg = "Encoding: "+ img.encoding;

  if (DEBUG){
    ROS_INFO("-----------------------------------------------------");
    ROS_INFO("image received: %d x %d, data_size: %d", img_width, img_height, img_num_pixels);
  }

  // not a perfect strategy. Only check for the pixel locations on the first row where they are found
  // EG dont bother checking all the rows that conatin white.
  // potentially flawed but staisfies the rubric
  int firstPixel_x;
  int lastPixel_x;
  int midPixel_x;
  bool leftPixelFound = false;
  bool rightPixelFound = false;

  int leftWhitePixel_y;
  int leftWhitePixel_x;
  int rightWhitePixel_y;
  int rightWhitePixel_x;

  int rightWhitePixel_index;
  int leftWhitePixel_index;

  int centreBall_index;
  int centreBall_x;

  // loop through all the elements in the data array until we find a white midPixel
  // dont reshape the image into 2D matrix as not necessary. Find the first white pixel first
  // and work from there
  for (int i = 0; i < img_num_pixels; i++) {
    // parse each colour in the pixel
    // note that each element in the data array is a colour channel value for each pixel
    // ie each pixel is represented by 3 elements
    R = img.data[i*3];
    G = img.data[i*3+1];
    B = img.data[i*3+2];

    if ((R == 255) && (G == 255) && (B == 255)) {

      leftPixelFound = true;
      // find out what row this is on.
      leftWhitePixel_y = i / img_width;
      leftWhitePixel_x = i % img_width;
      leftWhitePixel_index = i;
      if (DEBUG) {
        ROS_INFO("found left pixel: %d \t: x:%u, y:%u", i, leftWhitePixel_x, leftWhitePixel_y);
      }
      break;
    }
  }

  // calculate the index of the end of the row
  int endOfRow = (leftWhitePixel_y + 1 )* img_width;

  // ROS_INFO("find last white pixel: left: %u, endofRow: %u" , leftWhitePixel_index, endOfRow);
  // pick up where we left off, and find the right hand side of the ball
  // find the next non white pixel
  if (leftPixelFound) {
    // ROS_INFO("find last white pixel: left: %u, endofRow: %u" , leftWhitePixel_index, endOfRow);
    for (int i = leftWhitePixel_index ; i < endOfRow; i++) {
      R = img.data[i*3];
      G = img.data[i*3+1];
      B = img.data[i*3+2];

      if ((R != 255) && (G != 255) && (B != 255)) {
        rightPixelFound = true;
        rightWhitePixel_y = i / img_width;
        rightWhitePixel_x = i % img_height-1;
        rightWhitePixel_index = i - 1;
        if (DEBUG) {
          ROS_INFO("right pixel (non white): %d \t: R:%u, G:%u, B:%u", i, R, G, B);
        }
        break;
      }
      else {
        // temp for debugging, can be removed but does not harm
        rightWhitePixel_y = i / img_width;
        rightWhitePixel_x = i % img_height-1;
        rightWhitePixel_index = i - 1;
        if (DEBUG) {
          ROS_INFO("right pixel (white): %d \t: R:%u, G:%u, B:%u", i, R, G, B);
        }
      }
    } // end of the loop through remaining pixels on rows

    // if no non white pixel was found after the white pixel(s), then use the max image width as the edge of the ball
    // execute after checking each pixel, if no pixel was found
    if (!rightPixelFound) {
      rightWhitePixel_x = rightWhitePixel_index % img_width-1;
      // rightWhitePixel_index = i - 1;
      if (DEBUG) {
        ROS_INFO("right side of ball not found, using last pixel, i:%u", rightWhitePixel_x) ;
      }
    }

    centreBall_x = leftWhitePixel_x + (rightWhitePixel_x - leftWhitePixel_x) / 2;
    if (DEBUG) {
      ROS_INFO("right pixel: %d \t:, rightWhitePixel: %u", rightWhitePixel_index, rightWhitePixel_x);
      ROS_INFO("ball centre x: %u", centreBall_x);
      ROS_INFO("-----------------------------------------------------");
      ROS_INFO("-----------------------------------------------------");
    }
  }
  else {
    // issue stop
    if (DEBUG) {
      ROS_INFO("no ball found");
    }
    // set ball location to -1 (implausible pixel location)
    centreBall_x = -1;
  }

  // TODO: Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera
  // determine_command(centreBall_x);
  float lin_x;
  float ang_z;
  std::tie(lin_x, ang_z) = determine_command(centreBall_x);
  if (DEBUG) {
    ROS_INFO("linear_x:%1.2f, angular_z: %1.2f", lin_x, ang_z);
  }
  drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv)
{
  ROS_INFO("Ready to process images...");
  
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}
