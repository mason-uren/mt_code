
# Dynamic Extrinsics (Offline)  
## Objective:  
// TODO  	

## Contents:
- [Dependencies](#dependencies)
- [Files](#files)
- [How to build](#build)
- [Running the application](#run)
- [Examples](#examples)
	
## Dependencies  
- cmake (v3.15) ⇐ at least version 3.5
- Eigen (v3.3)  
- [HDF5](https://www.hdfgroup.org/downloads/hdf5/source-code/) (v1.12)  
- OpenCV (v3.4) ⇐ TODO upgrade to v4.2  
- [nholmann::json](https://github.com/nlohmann/json) (v3.2)  
- [L-BFGS++](https://lbfgspp.statr.me)  
  
## Files  
├── DataAcquisition  
│   ├── H5Parser.h  
│   └── Preprocess.h  
├── DynamicExtrinsics  
│   ├── Extrinsics.h  
│   └── ModelError.h  
├── main.cpp  
├── Matrices  
│   ├── Matrix3d.h  
│   └── Transformations.h  
├── Models  
│   ├── Board  
│   │   └── FiducialModel.h  
│   ├── Camera  
│   │   └── CameraModel.h  
│   └── PanTilt  
│       └── PanTiltModel.h  
└── README.md  
  
## Build  
  
Once the system dependencies are met, we can proceed with building. This stand-alone application follows CMake best practices and as such can be built by creating `build/` at the `CMakeList.txt` root, then running the following from said directory:
```
cmake <cmake-options> ..
```

The resulting binary is called `Metrology2020_Algorithms_OBJ`.

## Run
To run the application we simply call, `./Metrology2020_Algorithms_OBJ`. Successful program completion should report the new model to `console`, save a record to:
```
dev/DynamicExtrinsics/python/cad_models/cpp-2019-09-04-cad-model.json
```

Example successful console output:
```
Alg. Iterations: ( 1)
x: (  -0.424565   0.064487   0.060751 -0.0375593    0.20123  0.0150479)
f(x): ( 0.584728)
PTU Model Shape: ( 16, 1)
rz_ptu 0.0150479
ry_ptu 0.20123
rx_ptu -0.0375593
z_ptu 0.060751
rx_camera -0.000988502
z_offset 0
y_ptu 0.064487
rz_offset 0
x_camera 0.17328
ry_camera -0.137918
z_camera -0.120351
rz_camera -0.0132186
y_camera 0.115606
pan_scale 1
tilt_scale 1
x_ptu -0.424565
```

## Examples
### Quick Tips:
1. Common data type transformations.
```
	#include <Eigen/Core>
	#include <opencv2/core/core.hpp>
	#include <opencv2/core/eigen.hpp>

	// Note: Recommended to use the Eigen::Matrix "typename" specified by the algorithms module <E_Matrix>. This assumes matrix storage order is rowMajor (like OpenCV).

	int main() {
		/* 
		 * cv::Mat -> Eigen::Matrix
		 * Given: (type) cv::Mat (obj-name) cvMat
		 */
		auto eMat{typename Matrix3d<Primitive, Key>::E_Matrix(cvMat.rows, cvMat.cols)};
		cv::cv2eigen(cvMat, eMat);
		
		/*
		 * Eigen::Matrix -> cv::Mat
		 * Given: (type) Eigen::Matrix (obj-name) eMat
		 */
		 auto cvMat{cv::Mat{}}; // Have found that we don't need to specify size initially
		 cv::eigen2cv(eMat, cvMat);
	 }
 ```
 
 2. Loading data
		 
 ``` 
	#include <DynamicExtrinsics/Matrix3d.h>
	
	int main() {
			/*
			 * Raw data -> Eigen::Matrix (Matrix3d)
			 * Given: (type) T * (obj-name) rawPtr;
			 * 
			 * Note: We recommend using the Matrix3d<Primitive, Key> class for raw data -> eigen transformations.
			 */
			 auto metrics{Matrix3d<Primitive, Key>{<optional-arg> storage order (Default: rowMajor)}}; // No size needed
		 
			 // Allocated memory for raw pointer transfer (ie size information)
			 auto key{<buffer-id>}
			 vector<unsigned long long> dimensions{};
			 metrics.createBuffer(key, dimensions, <optional-arg> isRowMajor (Default: rowMajor));
			 metrics.mapMatrix(rawPtr, key, <optional-arg> isRowMajor (Default: rowMajor));

			// OR
			// Create key (no memory allocation, but data-buffers assocaited with Matrix3d are not created)
			metrics.addKey(key, rows, cols, <optional-arg> flatten (Default: false));
		
			/*
			 * Eigen::Matrix (Matrix3d) -> Raw Data
			 * Given: (type) Matrix3d<Primitive, Key> (obj-name) metrics
			 */
			 auto rawPtr{metrics.getBuffer(key)->data}; // Reference to underlying rawPtr; only if allocated using createBuffer()
			
			// OR
			// If matrix was created via addKey(), recommend the following...
			auto rawPtr{metrics().data()}; // Direct eigen matrix raw ptr access; Assumes metrics has only one key
			
			// OR
			//  auto rawPtr{metrics[key].data()}; // Direct Eigen::Matrix raw ptr access: Needs dataset key for access
	}
```

3. PTU Model (load/save model)

```
	#include <DynamicExtrinsics/PanTiltModel.h>
	
	// Models (file paths are not complete)
	constexpr char PTU_MODEL_JSON[] = "dev/DynamicExtrinsics/python/cad_models/2019-09-04-cad-model.json";  
	constexpr char CPP_GEN_MODEL[] = "dev/DynamicExtrinsics/python/cad_models/cpp-2019-09-04-cad-model.json";
	
	int main() {
		// Create model 
		auto PTUModel{Model::PanTilt<datatype, Set::Key>()};  

		// Load from file (ie PTU_MODEL_JSON)
		PTUModel.loadModel(PTU_MODEL_JSON);  
		PTUModel.displayModel();  

		// Save to file (ie CPP_GEN_MODEL)
		PTUModel.saveModel(CPP_GEN_MODEL);
	}
```

4. Fit params to model

```

	int main() {
		// Parameters to fit
		std::vector<Set::Key> paramsToFit{  
			 Set::CadModelTypes::X_PTU, Set::CadModelTypes::Y_PTU, Set::CadModelTypes::Z_PTU,  
			 Set::CadModelTypes::RX_PTU, Set::CadModelTypes::RY_PTU, Set::CadModelTypes::RZ_PTU  
		};  

		/*
		 * Fit Static Params
		 * Model is fit against params using loss function while incrementally being updated along the target parameters.
		 */
		DynamicExtrinsics::fitStaticParams<datatype, Set::Key>(
			PTUModel,  			// model to fit against (doesn't necessarily need to be PTU, just match the same format)
			extrinsics,  		// loaded datasets (type) Matrix3d<>
			paramsToFit,  		// parameters to fit against (type) vector<string>
			&ModelError::staticExplodedLoss<datatype, Set::Key>); // Loss function callback
  }
```

## Developers' Notes
### Issues/Concerns:
- Undistort Camera Corners: 

	<b>Issue</b>:  Data returned from the function `cv::undistortPoints(inBuf, outBuf, intrscBuf, distrBuf);`does not exactly math python implementation of same function.
	
	<I>Description:</I> Given the same `cv::Mat` input matrices:
	```
	inBuf : "UV_corners_<camera>"
	intrscBuf : "camera instrisics"
	distrBuf: "camera distortions"
	```
	the resultant `outBuf` always differs from the python result:

	```
	| UV_corners_ximea[0][0][0]|
	| python | c++ |
	|--------|-----|
	|-0.08407642 |-0.08405190202231248 |
	```

	The values differ by ~ 100,000th of a meter, but when dealing with 			sub-millimeter precision this may be an issue. 

- L-BFGS Minimization:

	<b>Issue:</b> (or Suspected Issue). The minimization algorithm responsible for updating the PTU model only goes through one iteration...
	
	<I>Description:</I> The number of iterations it takes to minimize our objective vector `x` is only 1, which although is the same number as the python code, "smells" suspect. Further investigation is warranted!

- Rotation Vector ⇒ Rotation Matrix Generation 

	<b>Issue:</b> Generated rotation matrix doesn't match python equivalent:

	<I>Data (Env - Linux)</I>

	Given:
	```
	Rotation Vector:
	-0.0375593
	 0.20123
	 0.0150479
	```

	Result:
	  
	  
	```  
	c++  - "Eigen::EulerAngles"
	 0.979711   -0.0147437   0.199874  
	 0.00753215  0.999295    0.0367928  
	-0.200276   -0.0345408   0.97913  
	
	c++ - "OpenCV::Rodrigues"
	 0.979711   -0.0187082   0.19953808
	 0.011176    0.9991842   0.03880489
	-0.200101   -0.0357874   0.97912140 
	  
	python - "transform3d.euler2mat"
	 0.9797106  -0.02254122  0.19914576   
	 0.01474367  0.99906866  0.04055171   
	-0.19987437 -0.0367928   0.9791305   
	```
		


