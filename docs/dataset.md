# Dataset

The `vswarm` drone detection dataset comprises 11822 images (9891 for training, 1931 for validation) recorded in five different environments around the [EPFL campus](https://www.epfl.ch/).
The images are taken from a static camera and automatically labeled using background subtraction (modulo some minor data cleaning).

The following camera hardware was used to acquire the images:

- A 1.6MP grayscale USB3 camera: [FLIR Firefly S](https://www.flir.eu/products/firefly-s/?model=FFY-U3-16S2M-S) Model `FFY-U3-16S2M-S`
- A fisheye lens: [OpenMV Ultra Wide Angle Lens](https://openmv.io/products/ultra-wide-angle-lens)

The [tree](https://en.wikipedia.org/wiki/Tree_(command)) structure of the dataset is as follows:

```
vswarm_drone_detection_dataset
├── <environment-name>_<flight-#>
│   ├── images
│   │   ├── frame_<seq-#>_<unix-timestamp>.png   ---> Image file
│   │   ├── frame_<seq-#>_<unix-timetsamp>.json  ---> Label file (LabelMe format)
│   │   ├── ...
│   ├── labels
│   │   ├── frame_<seq-#>_<unix-timestamp>.txt   ---> Label file (Darknet format)
│   │   ├── ...
├── ...
├── README.md  ---> This README
├── train.txt  ---> Paths of images used for training (9891 examples)
├── valid.txt  ---> Paths of images used for validation (1931 examples)
```

We provide labels in two different formats.

## Darknet format

The [Darknet](https://github.com/pjreddie/darknet) format is `.txt`-based and mainly useful for training/testing the detector.

Each `.txt` file has the following format (one detection per line):

```
<class-id> <center-x> <center-x> <width> <height>
```

where:

- `<class-id>` is the zero-indexed ID of the class (only one in our case: `jetson_lequad`)
- `<center-x/y>` is the normalized (range 0-1) center of the bounding box in x/y-direction
- `<width/height>` is the normalized (range 0-1) width/height of the bounding box

Example:

```
0 0.436806 0.699074 0.123611 0.150000
```

## LabelMe format

The [LabelMe](https://github.com/wkentaro/labelme) format is `.json`-based and mainly useful for reviewing the auto-generated labels and/or manual labeling.

Each `.json` file has the following format:

```
{
    "fillColor": [255, 0, 0],
    "flags": {},
    "imageData": null,
    "imageHeight": 540,
    "imagePath": "frame_075_1599398126643726885.png",
    "imageWidth": 720,
    "lineColor": [0, 255, 0, 128],
    "shapes": [
        {
            "fill_color": null,
            "flags": {},
            "label": "jetson_lequad",
            "line_color": null,
            "points": [[270, 337], [359, 418]],
            "shape_type": "rectangle"
        }
    ],
    "version": "3.16.2"
}
```

where `"points"` specifies the top left and bottom right extreme point of the bounding box and `"label"` the class ID.
