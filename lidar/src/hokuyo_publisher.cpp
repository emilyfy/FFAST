/* HokuyoAIST
 *
 * Laser data read example.
 *
 * Copyright 2008-2011 Geoffrey Biggs geoffrey.biggs@aist.go.jp
 *     RT-Synthesis Research Group
 *     Intelligent Systems Research Institute,
 *     National Institute of Advanced Industrial Science and Technology (AIST),
 *     Japan
 *     All rights reserved.
 *
 * This file is part of HokuyoAIST.
 *
 * HokuyoAIST is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License,
 * or (at your option) any later version.
 *
 * HokuyoAIST is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with HokuyoAIST. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <flexiport/flexiport.h>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/hokuyo/data_scan.hpp"

int main(int argc, char **argv)
{
    std::string port_options("type=serial,device=/dev/Hokuyo,timeout=1");
    double start_angle(0.0), end_angle(0.0);
    int first_step(-1), last_step(-1);
    int multiecho_mode(0);
    unsigned int speed(0), cluster_count(1);
    bool get_intensities(false), get_new(false), verbose(false);
    float prev_time(0.0);

    int opt;
    // Get some options from the command line
    while((opt = getopt(argc, argv, "b:c:e:f:il:m:no:s:u:vh")) != -1)
    {
        switch(opt)
        {
            case 'c':
                sscanf(optarg, "%d", &cluster_count);
                break;
            case 'e':
                sscanf(optarg, "%lf", &end_angle);
                break;
            case 'f':
                sscanf(optarg, "%d", &first_step);
                break;
            case 'i':
                get_intensities = true;
                break;
            case 'l':
                sscanf(optarg, "%d", &last_step);
                break;
            case 'm':
                sscanf(optarg, "%d", &speed);
                break;
            case 'n':
                get_new = true;
                break;
            case 'o':
                port_options = optarg;
                break;
            case 's':
                sscanf(optarg, "%lf", &start_angle);
                break;
            case 'u':
                sscanf(optarg, "%d", &multiecho_mode);
                break;
            case 'v':
                verbose = true;
                break;
            case '?':
            case 'h':
            default:
                std::cout << "Usage: " << argv[0] << " [options]\n\n";
                std::cout << "-c count\tCluster count.\n";
                std::cout << "-e angle\tEnd angle to get ranges to.\n";
                std::cout << "-f step\t\tFirst step to get ranges from.\n";
                std::cout << "-i\t\tGet intensity data along with ranges.\n";
                std::cout << "-l step\t\tLast step to get ranges to.\n";
                std::cout << "-m speed\tMotor speed.\n";
                std::cout <<
                    "-n\t\tGet new ranges instead of latest ranges.\n";
                std::cout <<
                    "-o options\tPort options (see flexiport library).\n";
                std::cout << "-s angle\tStart angle to get ranges from.\n";
                std::cout << "-u mode\tMulti-echo detection:\n";
                std::cout << "\t\t0: Off (default), 1: Front, 2: Middle, "
                    "3: Rear, 4: Average\n";
                std::cout <<
                    "-v\t\tPut the hokuyoaist library into verbose mode.\n";
                return 1;
        }
    }

    try
    {
        hokuyoaist::Sensor laser; // Laser scanner object
        // Set the laser to verbose mode (so we see more information in the
        // console)
        if(verbose)
        {
            laser.set_verbose(true);
        }

        // Open the laser
        laser.open(port_options);

        // Calibrate the laser time stamp
        // std::cout << "Calibrating laser time\n";
        laser.calibrate_time();
        // std::cout << "Calculated offset: " << laser.time_offset() << "ns\n";
        // std::cout << "Calculated drift rate: " << laser.drift_rate() << '\n';
        // std::cout << "Calculated skew alpha: " << laser.skew_alpha() << '\n';

        // Turn the laser on
        laser.set_power(true);
        // Set the motor speed
        try
        {
            laser.set_motor_speed(speed);
        }
        catch(hokuyoaist::MotorSpeedError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        catch(hokuyoaist::ResponseError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        // Set multiecho mode
        switch(multiecho_mode)
        {
            case 1:
                laser.set_multiecho_mode(hokuyoaist::ME_FRONT);
                break;
            case 2:
                laser.set_multiecho_mode(hokuyoaist::ME_MIDDLE);
                break;
            case 3:
                laser.set_multiecho_mode(hokuyoaist::ME_REAR);
                break;
            case 4:
                laser.set_multiecho_mode(hokuyoaist::ME_AVERAGE);
                break;
            case 0:
            default:
                laser.set_multiecho_mode(hokuyoaist::ME_OFF);
                break;
        }

        lcm::LCM lcm;
        if(!lcm.good()) {
            std::cout << "Failed to open LCM." << std::endl;
            return 1;
        }
        
        hokuyo::data_scan my_data;

        while(1)
        {
            // Get some laser info
            // std::cout << "Laser sensor information:\n";
            hokuyoaist::SensorInfo info;
            laser.get_sensor_info(info);
            // std::cout << info.as_string();

            // Get range data
            hokuyoaist::ScanData data;
            laser.get_ranges(data, -1, -1, cluster_count);
            /*
            if((first_step == -1 && last_step == -1) &&
                (start_angle == 0.0 && end_angle == 0.0))
            {
                // Get all ranges
                if(get_new)
                {
                    laser.get_new_ranges(data, -1, -1, cluster_count);
                }
                else if(get_intensities)
                {
                    laser.get_new_ranges_intensities(data, -1, -1, cluster_count);
                }
                else
                {
                    laser.get_ranges(data, -1, -1, cluster_count);
                }
            }
            else if(first_step != -1 || last_step != -1)
            {
                // Get by step
                if(get_new)
                {
                    laser.get_new_ranges(data, first_step, last_step,
                            cluster_count);
                }
                else if(get_intensities)
                {
                    laser.get_new_ranges_intensities(data, first_step, last_step,
                            cluster_count);
                }
                else
                {
                    laser.get_ranges(data, first_step, last_step, cluster_count);
                }
            }
            else
            {
                // Get by angle
                if(get_new)
                {
                    laser.get_new_ranges_by_angle(data, start_angle, end_angle,
                            cluster_count);
                }
                else if(get_intensities)
                {
                    laser.get_new_ranges_intensities_by_angle(data, start_angle,
                            end_angle, cluster_count);
                }
                else
                {
                    laser.get_ranges_by_angle(data, start_angle, end_angle,
                            cluster_count);
                }
            }
            */

            // std::cout << "Measured data:\n";
            // std::cout << data.as_string();

            my_data.timestamp = info.time*0.001;
            my_data.angle_min = info.min_angle;
            my_data.angle_max = info.max_angle;
            my_data.angle_increment = info.resolution;
            my_data.time_increment = info.time_resolution*0.001;
            my_data.scan_time = my_data.timestamp - prev_time;
            prev_time = my_data.timestamp;
            my_data.range_min = info.min_range*0.001;
            my_data.range_max = info.max_range*0.001;
            my_data.ranges_length = data.ranges_length();
            my_data.intensities_length = data.intensities_length();
            my_data.ranges.resize(my_data.ranges_length);
            my_data.intensities.resize(my_data.intensities_length);
            const uint32_t* r_ptr = data.ranges();
            const uint32_t* i_ptr = data.intensities();
            for (int i=0;i<my_data.ranges_length;i++) my_data.ranges[i] = *(r_ptr+i) * 0.001;
            for (int i=0;i<my_data.intensities_length;i++) my_data.intensities[i] = *(i_ptr+i);

            // std::cout << "publishing data to lcm" << std::endl;
            lcm.publish("Hokuyo", &my_data);
        }

        // Close the laser
        laser.close();
    }
    catch(hokuyoaist::BaseError &e)
    {
        std::cerr << "Caught exception: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

