# OC-SORT 
## C/C++ Implementation

A C/C++ implementation that uses static memory allocation, no vector library, no third parties library to manage matrices.

run:
``` bash
./main dets/test-dets.bin
```

1. `test-dets.bin` is a binary file of detections pre-computed using YOLO11s on, this particular file is the sequence 0016 of the test KITTI dataset. This could be a python script that uses the ultralytics library. It's a direct dump of numpy in the following format:

|            | Frame ID | Bounding box * | Confidence | Class |
|------------|----------|----------------|------------|-------|
| data type  | float    | float          | float      | float |
| byte count | 4        | 4x4            | 4          | 4     |

\*Bounding Box xyxy


