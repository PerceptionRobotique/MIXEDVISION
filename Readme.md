# MIXEDVISION: Multiple-camera types simultaneous stereo calibration software

## Dependencies:
- libxml2 (2.9.13 tested)
  sudo apt install libxml2-dev

- OpenCV (4.2.0 tested)

- GSL (23.1.0 tested) for CRing
	sudo apt-get install libgsl-dev
	
- ViSP (3.4.1 tested) with updates to vpDot2 member variables and methods from private to protected in sourcecode for CRing:
```
protected:
	virtual bool hasGoodLevel(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v) const;

	bool findFirstBorder(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                       unsigned int &border_u, unsigned int &border_v);
  
  /*!

  Get the starting point on a dot border. The dot border is
  computed from this point.

  \sa getFirstBorder_v()

  */
  unsigned int getFirstBorder_u() const { return this->firstBorder_u; }
  /*!

  Get the starting point on a dot border. The dot border is
  computed from this point.

  \sa getFirstBorder_u()

  */
  unsigned int getFirstBorder_v() const { return this->firstBorder_v; }
  
  bool isInArea(const unsigned int &u, const unsigned int &v) const;
  
  bool computeFreemanChainElement(const vpImage<unsigned char> &I, const unsigned int &u, const unsigned int &v,
                                  unsigned int &element);
    void computeFreemanParameters(const int &u_p, const int &v_p, unsigned int &element, int &du, int &dv, float &dS,
                                float &dMu, float &dMv, float &dMuv, float &dMu2, float &dMv2);
  
  // flag
  bool compute_moment; // true moment are computed
  
  // Area where the dot is to search
  vpRect area;
  
  // Bounding box
  int bbox_u_min, bbox_u_max, bbox_v_min, bbox_v_max;
```

or replace VISP_SOURCE_DIR/modules/tracker/blob/include/visp3/blob/vpDot2.h with MIXEDVISION's version MIXEDVISION/vispHeaderFileForCRing/vpDot2.h (made from ViSP 3.4.1).


## Basic example to calibrate a single omnidirectional camera from the command line tool:

This assumes MUI/calibrateSystem.cpp has only `#define O` defined. For the command line: 
```
cd PATH_TO_BUILT_MIXEDVISION/MUI/
cd ./CalibrateSystem -p PATH_TO_CALIBRATION_2023EventCameraCalib/fisheye/2023_04_10/ -f 1 -n 6 -t 1 -c -o
```
### Meaning of the options
| Option | Description |
|----|----|
|`-p`| path to image files which names have the form grid36-NN.png, where NN is a number from 00 to 99
|`-f`| number of the first image to consider, e.g. 1 if grid36-01.png is the first image to consider
|`-n`| the number of the last image to consider, e.g. 6 if the last image is grid36-06.png
|`-t`| calibration target type (0: dots; 1: chessboard; 2: rings)
|`-o`| outputs reprojection images as ppm files in the path to image files provided with `-p`
|`-c`| in case you already have the pointN.txt (e.g. point0.txt if single camera) file from previous clicks

Procedure to follow:
Once launched, the first image is displayed and you must click to the points corresponding to those defined in the file parametresMireN.txt (parametresMire1.txt in case of chessboard type) located at the same place as the CalibrateSystem binary and containing, for instance:
1
0.0334 0.0334 0.0
9 6 1
4
1 1 0
1 4 0
7 4 0
7 1 0

The meaning of each line is:
- the target type
- the X step, Y step, Z step between primitives in your own unit (it sets the units of the output translation)
- the X, Y and Z number of primitives
- the number of points to click to initialize the detection
- the X, Y, Z indexes expressed in the target frame of the first point to click
- the X, Y, Z indexes expressed in the target frame of the second point to click
- the X, Y, Z indexes expressed in the target frame of the third point to click
- the X, Y, Z indexes expressed in the target frame of the fourth point to click


```
Copyright (C) 2014-2023 by MIS lab (UPJV). All rights reserved.

See http://mis.u-picardie.fr/~g-caron/fr/index.php?page=7 for more information.

This software was developed at:
MIS - UPJV
33 rue Saint-Leu
80039 AMIENS CEDEX
France

and updated since 2023 at:
CNRS - AIST JRL (Joint Robotics Laboratory)
1-1-1 Umezono, Tsukuba, Ibaraki
Japan

This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

Description:
Insight about how to set the project and build the program
Authors:
Guillaume CARON

```

