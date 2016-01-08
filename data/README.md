Test Data
=========

In order to run tests that depend on mock data,

1. Copy them to this folder respecting the following hierarchy.
2. Remove `DISABLED_` prefix in `CalibratorTest.cc`
3. Rebuild and run `test/tests`

		├── README.md
		└── test0
		    └── procam0
		        ├── baseline_color_frames
		        │   └── frame0.png
		        ├── baseline_depth_frames
		        │   └── frame0.yml
		        ├── depth_frames
		        ├── depth_variance
		        │   └── frame0.yml
		        ├── fullHD_frames
		        │   ├── frame0.png
		        │   ├── frame1.png
		        │   ├── frame2.png
		        │   ├── ...
		        │   └── frameN.png
		        ├── param.json
		        ├── undistorted_HD_frames
		        └── undistorted_frames
