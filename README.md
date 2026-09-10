# OC-SORT 
## C/C++ Implementation

A C/C++ implementation that uses static memory allocation, no vector library, no third parties library to manage matrices.

run:
``` bash
./main dets/test-dets.bin
```

## `test-dets.bin`
`test-dets.bin` is a binary file of detections pre-computed using YOLO11s on, this particular file is the sequence 0016 of the test KITTI dataset. This could be a python script that uses the ultralytics library. It's a direct dump of numpy in the following format:

|            | Frame ID | Bounding Box * | Confidence | Class |
|------------|----------|----------------|------------|-------|
| data type  | float    | float          | float      | float |
| byte count | 4        | 4x4            | 4          | 4     |

\*Bounding Box Format: *xyxy*

An example script how this detection file is generated can be found on `dets/ex-generated_detections.py`

