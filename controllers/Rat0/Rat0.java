// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import com.cyberbotics.webots.controller.PositionSensor; //ADDED

import java.util.Random;

public class Rat0 extends Robot {

  protected final int timeStep = 32;
  protected final double maxSpeed = 800;
  protected final double superSpeed = 1000; //ADDED
  protected final double[] collisionAvoidanceWeights = {0.1,0.03,0.015,0.0,0.0,-0.015,-0.03,-0.06};
  //                                                        0    1      2      3    4   5     6   7
  // protected final double[] collisionAvoidanceWeights = {-0.002,-0.005,-0.015,-0.6,0.3,0.005,0.0,0.0};{-0.002,-0.005,-0.015,-0.6,0.3,0.005,0.0,0.0};
  //                                                    0   1   2   3   4   5   6   7
  // protected final double[] collisionAvoidanceWeights = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  // protected final double[] slowMotionWeights = {0.0125,0.00625,0.0,0.0,0.0,0.0,0.00625,0.0125};
  protected final double[] slowMotionWeights = {0.8,0.1,0.0,0.0,0.0,0.0,0.1,0.8};

  //ADDED
  protected final double wheelRadius = 0.02;
  protected final double axleLength = 0.052;

  protected Accelerometer accelerometer;
  protected Camera camera;
  protected int cameraWidth, cameraHeight;
  protected Motor leftMotor, rightMotor;
  protected DistanceSensor[] distanceSensors = new DistanceSensor[8];
  protected LightSensor[] lightSensors = new LightSensor[8];
  protected LED[] leds = new LED[10];

  //ADDED
  protected PositionSensor lps;
  protected PositionSensor rps;
  protected double ldis = 0;
  protected double rdis = 0;
  protected double dori = 0;

  public Rat0() {
    accelerometer = getAccelerometer("accelerometer");
    camera = getCamera("camera");
    camera.enable(8*timeStep);
    cameraWidth=camera.getWidth();
    cameraHeight=camera.getHeight();
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    for (int i=0;i<10;i++) {
      leds[i]=getLED("led"+i);
    };
    for (int i=0;i<8;i++) {
      distanceSensors[i] = getDistanceSensor("ps"+i);
      distanceSensors[i].enable(timeStep);
      lightSensors[i] = getLightSensor("ls"+i);
      lightSensors[i].enable(timeStep);
    }
    batterySensorEnable(timeStep);

    //ADDED
    lps = leftMotor.getPositionSensor(); //left_position_sensor
    rps = rightMotor.getPositionSensor(); //right_position_sensor
    lps.enable(64);
    rps.enable(64);

  }

  public void run() {

    int blink = 0;
    int oldDx = 0;
    // Random r = new Random();
    boolean turn = false;
    boolean right = false;
    boolean seeFeeder = false;
    double battery;
    double oldBattery = -1.0;
    int image[];
    double distance[] = new double[8];
    int ledValue[] = new int[10];
    double leftSpeed, rightSpeed;

    //ADDED
    int timer = 0;
    boolean flag1 = false;
    boolean flag2 = false;
    boolean flag3 = false;

    while (step(timeStep) != -1) {

      // read sensor information
      image = camera.getImage();
      for(int i=0;i<8;i++) distance[i] = distanceSensors[i].getValue();
      // battery = batterySensorGetValue();
      for(int i=0;i<10;i++) ledValue[i] = 0;

      // obstacle avoidance behavior
      leftSpeed  = maxSpeed;
      rightSpeed = maxSpeed;

      for (int i=0;i<8;i++) {
        leftSpeed  -= (slowMotionWeights[i]+collisionAvoidanceWeights[i])*distance[i];
        rightSpeed -= (slowMotionWeights[i]-collisionAvoidanceWeights[i])*distance[i];
      }
      // return either to left or to right when there is an obstacle
      // if (distance[6]+distance[7] > 1800 || distance[0]+distance[1] > 1800) {

      /*
         |    |  ` , __     ___
         |\  /|  | |'  `. .'   `
         | \/ |  | |    | |----'
         /    /  / /    | `.___,
      */
      // for(int i=0;i<8;i++) System.out.println("distance["+i+"] == "+ distance[i]);
      /*
         ___      _                      _
        / _ \  __| | ___  _ __ ___   ___| |_ _ __ _   _
       | | | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |
       | |_| | (_| | (_) | | | | | |  __/ |_| |  | |_| |
        \___/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |
                                                  |___/
      */

    double l = lps.getValue();//wb_position_sensor_get_value(left_position_sensor);
    double r = rps.getValue();//wb_position_sensor_get_value(right_position_sensor);
    ldis = l * wheelRadius;         // distance covered by left wheel in meter
    rdis = r * wheelRadius;         // distance covered by right wheel in meter
    dori = (rdis - ldis) / axleLength;  // delta orientation

    /*
      _____
     |_   _|   _ _ __ _ __
       | || | | | '__| '_ \
       | || |_| | |  | | | |
       |_| \__,_|_|  |_| |_|

    */

      if (distance[0]+distance[7] > 900 && distance[5] > distance[2] || flag1) {
        System.out.println("Turn Right:"+timer);
        if (timer++ >= 1) {
          leftSpeed  = maxSpeed;
          rightSpeed = -maxSpeed;
          ledValue[8] = 1;
          flag1 = true;
          if (timer == 15){
           timer = -5;
           leftSpeed  = maxSpeed;
           rightSpeed = maxSpeed;
           ledValue[8] = 0;
           flag1 = false;
          }
        }
      }
      else if (distance[0]+distance[7] > 900 && distance[2] > distance[5] || flag2) {
        System.out.println("Turn Left:"+timer);
        if (timer++ >= 1) {
          leftSpeed  = -maxSpeed;
          rightSpeed = maxSpeed;
          ledValue[8] = 1;
          flag2 = true;
          if (timer == 15){
           timer = -5;
           leftSpeed  = maxSpeed;
           rightSpeed = maxSpeed;
           ledValue[8] = 0;
           flag2 = false;
          }
        }
      }
      else if (distance[0]+distance[7] > 900 && (distance[2] >= distance[5] - 150 && distance[2] <= distance[5] + 150) || flag3) {
        System.out.println("U-Turn:"+timer);
        if (timer++ >= 1) {
          leftSpeed  = -maxSpeed;
          rightSpeed = maxSpeed;
          ledValue[8] = 1;
          flag3 = true;
          if (timer == 15){
           timer = -5;
           leftSpeed  = maxSpeed;
           rightSpeed = maxSpeed;
           ledValue[8] = 0;
           flag3 = false;
          }
        }
      }

      if (blink++ >= 5) { // blink the back LEDs
        ledValue[6] = 0;
        if (blink == 20) blink = 0;
      }

      //LED mapping 8:body-G led0' to 'led7' (e-puck ring), 'led8' (body-G) and 'led9' (front)

      // set actuators
      for(int i=0; i<10; i++) {
        leds[i].set(ledValue[i]);
      }
      leftMotor.setVelocity(0.00628 * leftSpeed);
      rightMotor.setVelocity(0.00628 * rightSpeed);
    }
    // Enter here exit cleanup code
  }

  // public void compute_odometry( PositionSensor left_position_sensor, PositionSensor right_position_sensor) {
  //   double l = left_position_sensor.getValue();//wb_position_sensor_get_value(left_position_sensor);
  //   double r = right_position_sensor.getValue();//wb_position_sensor_get_value(right_position_sensor);
  //   double dl = l * wheelRadius;         // distance covered by left wheel in meter
  //   double dr = r * wheelRadius;         // distance covered by right wheel in meter
  //   double da = (dr - dl) / axleLength;  // delta orientation
  //   System.out.println("estimated distance covered by left wheel: "+dl+" m.\n");
  //   System.out.println("estimated distance covered by right wheel: "+dr+" m.\n");
  //   System.out.println("estimated change of orientation: "+da+" rad.\n");
  // }

  // public void turn(){
  //   System.out.println("Turn");
  //   compute_odometry(leftMotor.getPositionSensor(), rightMotor.getPositionSensor());
  //   if (timer++ >= 40) { // blink the back LEDs
  //     leftSpeed  = 0;
  //     rightSpeed = 0;
  //     ledValue[7] = 1;
  //     if (timer == 50){
  //      timer = 0;
  //      leftSpeed  = maxSpeed;
  //      rightSpeed = maxSpeed;
  //      ledValue[7] = 0;
  //     }
  //   }
  // }

  public static void main(String[] args) {
    Rat0 rat0 = new Rat0();
    rat0.run();

  }
}
