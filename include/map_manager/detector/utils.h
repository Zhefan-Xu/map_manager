 /*
 	FILE: utils.h
 	--------------------------
 	function utils for detectors
 */

#ifndef MAPMANAGER_DETECTOR_UTILS_H
#define MAPMANAGER_DETECTOR_UTILS_H

namespace mapManager{
    struct box3D
    {
        /* data */
        float x, y, z;
        float x_width, y_width, z_width;
        float id;
        float Vx, Vy;
        float Ax, Ay;
        bool is_dynamic=false; // false: not detected by yolo as dynamic, true: detected by yolo
    };
}

#endif