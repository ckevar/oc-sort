
# Compilation:

``` bash
gcc -O3 dets/generate_detections_synthetics.c -o gen_dets -DWAYMOv2_CLASS_DISTRIBUTION
```

Where WAYMOv2\_CLASS\_DISTRIBUTION compiles the generator using the WaymoV2 class distribution. It is also possible to generate using synthetics using MOT17 class detribution (MOT17\_CLASS\_DISTRIBUTION) and KITTI (KITTI\_CLASS\_DISTRIBUTION).
