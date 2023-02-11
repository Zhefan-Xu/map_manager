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
        bool is_human=false; // false: not detected by yolo as dynamic, true: detected by yolo
        bool is_dynamic=false; // false: not detected as dynamic(either yolo or classificationCB), true: detected as dynamic
        bool fix_size=false; // flag to force future boxes to fix size
    };
}

#endif