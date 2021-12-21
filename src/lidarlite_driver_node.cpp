/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * MIT License                                                             *
 * @author      Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @maintainer  Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @website    https://fieldroboticslab.ece.northeastern.edu/              *
 * @copyright (c) 2019, Northeastern University Field Robotics Lab(NEUFRL),*
 *             All rights reserved.                                        *
 *                                                                         *
 * Permission is hereby granted, free of charge, to any person obtaining   *
 * copy of this software and associated documentation files                *
 * (the "Software"), to deal in the Software without restriction,          *
 * including *without limitation the rights to use, copy, modify, merge,   *
 * publish, distribute, sublicense, and/or sell copies of the Software,    *
 * and to permit persons to whom the Software is furnished to do so,       *
 * subject to the following conditions:                                    *
 *                                                                         *
 * The above copyright notice and this permission notice shall be included *
 * in all copies or substantial portions of the Software.                  *
 *                                                                         *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS *
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF              *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. *
 *                                                                         *
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR        *
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF          *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH *
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.              *
 *                                                                         *
 * copy-left Licenses of Dependencies used :                               *
 *                                                                         *
 * 1. This code is wrapped using slightly modified version of code         *
 * in JHLidarLite github repository Licensed Under MIT License. The local  *
 * copy of license is located at include/JHLidarLite/ .                    *
 * the original copy of repo's (MIT) license can be obtained from          *
 * https://github.com/jetsonhacks/JHLidarLite.git                          *
 *                                                                         *
 * 2. This repo uses slightly modified FIR filter code, sourced  from      *
 * https://github.com/pms67/HadesFCS/blob/master/Filtering/C%20Code/FIR.h  *
 * repository, The above repo is written by Philip M.                      *
 * (Salmony @ philsal.co.uk), is licensed under BSD 3-Clause License,      *
 * the copy of same is available in the include/fir_filter/fir_filter.h    *
 * file locally. The original copy can be obtained from the repo,          *
 * https://github.com/pms67/HadesFCS/blob/master/LICENSE                   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
#include "lidarlite_driver/lidarlite_driver.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv,"lidarlite_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "lldriver_ns/Lidarlite_driver", remap, nargv);
    
    ros::waitForShutdown(); //blocking call
	return 0;
}
