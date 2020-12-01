
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
├── ConfigParser  
│   └── DynamicExtrinsicsJSONParser.h/cpp
├── Examples
│   └── main.cpp  
├── Extrinsics  
│   ├── ModelError.h  
│   └── ModelFit.h  
├── Model 
│   ├── Preprocess.h
│   ├── Transformations.h    
│   └── PanTiltModel.h  
└── README.md  
  
## Build  
  
Once the system dependencies are met, we can proceed with building. This stand-alone application follows CMake best practices and as such can be built by creating `build/` at the `CMakeList.txt` root, then running the following from said directory:
```
cmake <cmake-options> ..
```

The resulting binary is called `DynamicExtrinsics_Example`.

## Run
To run the application we simply call, `./DynamicExtrinsics_Example`. Successful program completion should report the new model to `console`, save a record to:
```
dev/DynamicExtrinsics/python/cad_models/cpp-2019-09-04-cad-model.json
```

Example successful console output:
```
/home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/build-release-lime2/bin/DynamicExtrinsics_Example
Searching for Config/ directory...
Found: /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/Config
Path previously sourced ( /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/Config)
Path previously sourced ( /home.shared/mruren/Projects/LiMe/Trunk.Boeing/ClosedLoopMetrology/Code/src/Config)
*** Dynamic Fit ***
Params to fit: 
x_camera : 0
y_camera : 0
z_camera : 0
rx_camera : 0
ry_camera : 0
rz_camera : 0
Loss: 0.000975426
Alg. Iterations: ( 1000)
x: (   0.0392638   0.0193184 -0.00362699 -0.00989161  -0.0190828  0.00972066     -2.9993     1.43256    0.103887     4.73975    0.472653     2.43094    -2.94361     1.53176      0.1568     4.88612     0.48603     1.48358)
f(x): ( 0.000975426)
*** Static Fit ***
Params to fit:
x_ptu : 0
y_ptu : 0
z_ptu : 0
rx_ptu : 0
ry_ptu : 0
rz_ptu : 0
Loss: 0.0014568
Alg. Iterations: ( 138)
x: (    0.34286   0.230489    1.09861 -0.0205581    1.48091 -0.0150897)
f(x): ( 0.0014568)
PTU Model Shape: ( 16, 1)
rz_ptu -0.0150897
ry_ptu 1.48091
rx_ptu -0.0205581
z_ptu 1.09861
rx_camera -0.00989161
z_offset 0
y_ptu 0.230489
rz_offset 0
x_camera 0.0392638
ry_camera -0.0190828
z_camera -0.00362699
rz_camera 0.00972066
y_camera 0.0193184
pan_scale 1
tilt_scale 1
x_ptu 0.34286

Process finished with exit code 0
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
		


