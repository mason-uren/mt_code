/**
 * @file Metrics.h
 * @brief 3d representation of Eigen matrices
 *
 * @author Mason U'Ren
 * @email mruren@hrl.com
 */

#ifndef METROLOGY2020_ALGORITHMS_METRICS_H
#define METROLOGY2020_ALGORITHMS_METRICS_H

#include <utility>
#include <vector>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <memory>

// Eigen3
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Shared/DynamicExtrinsicConfig.h>
#include <Shared/Presets.h>

/**
 * @tparam Primitive - underlying Eigen::Matrix element type
 * @tparam Key - Eigen::Matrix accessor type
 */
template <typename Primitive = double, typename Key = std::string>
class Matrix3d {
private:
    /**
     * @struct Buffer
     * @brief Contains type, dimension, and raw ptr to data.
     *
     * Helper class to the Eigen::Map API.
     *      Holds information regarding the raw data pointer that is mapped into an Eigen::Matrix.
     */
    struct Buffer {
        DE::Set::Buffer::Type type{};
        std::vector<unsigned long long> dims{};
//        unsigned long long stride{};
        void * data;

        Buffer(DE::Set::Buffer::Type type, std::vector<unsigned long long> &dims) //, const unsigned long long & stride)
            : type(type), dims(dims), data(nullptr) {} // stride(stride),
        Buffer(DE::Set::Buffer::Type type, std::vector<unsigned long long> &dims, void * data) // const unsigned long long & stride,
            : type(type), dims(dims), data(data) {} // stride(stride)
    };

    /**
     * @fn ::Buffer::Type getType(const int & rank)
     * @brief Determine the raw data buffer type from the rank.
     * @param[in] rank Dimensionality of raw data pointer.
     * @return ::Buffer::Type of raw data buffer.
     */
    DE::Set::Buffer::Type getType(const int & rank) {
		switch (rank) {
			case 1: return DE::Set::Buffer::Type::VECTOR;
			case 2: return DE::Set::Buffer::Type::MATRIX_2d;
			case 3: return DE::Set::Buffer::Type::MATRIX_3d;
			default:
				std::cout << "Warning: Unrecognized rank: ( " << rank << "). Default: 2" << std::endl;
				return DE::Set::Buffer::Type::MATRIX_2d;
		}
    }

    bool isMat3d(const DE::Set::Buffer::Type & bType) {
        return bType == DE::Set::Buffer::Type::MATRIX_3d;
    }

    // FIXME - include write up
    template<typename Func>
    struct Visitor : Func {
        explicit Visitor(const Func & func) : Func(func) {}

        template<typename S,typename I>
        void init(const S & v, I i, I j) { return Func::operator()(v, i, j); }
    };

    template<typename Mat, typename Func>
    void Visit(const Mat & matrix, const Func & func)
    {
        Visitor<Func> visitor(func);
        matrix.visit(visitor);
    }

    /**
     * @var std::unordered_map<Key, std::shared_ptr<Buffer>> dataBuffers
     * @brief Map of shared raw data pointers
     *
     * Ensures raw data pointer read into Eigen::Matrix containers don't get destroyed when calling functions
     *      exit scope.
     */
    std::unordered_map<Key, std::shared_ptr<Buffer>> dataBuffers{};
    std::string storageOrder{};

public:
    explicit Matrix3d(const std::string & storageOrder = Presets::DataSet::StorageOrder::ROW_MAJ) : storageOrder(storageOrder) {}
    ~Matrix3d() = default;

    /**
     * @var typedef Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> E_Matrix
     * @brief Type definition for row-major Eigen matrices.
     */
    typedef Eigen::Matrix<Primitive, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> E_Matrix;
    /**
     * @var typedef Eigen::Map<E_Matrix> E_Map
     * @brief Type definition for mapping raw data buffer memory regions to Eigen matrices.
     *
     * Developer's Note: Maps the existing memory region into Eigen structures.
     *      Need to handle life-cycle of raw data buffer that is mapped.
     *      <a href="https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html">Eigen Mapping Raw Buffers</a>
     */
    typedef Eigen::Map<E_Matrix> E_Map;

    /**
     * @vav typedef Eigen::Array<Primitive, Eigen::Dynamic> E_Array
     * @brief Type definition for Eigen array (similar to vector)
     */
    typedef Eigen::Array<Primitive, Eigen::Dynamic, Eigen::Dynamic> E_Array;

    /**
     * @fn static Metrics<Primitive, Key> createIdentityMatrices(const std::vector<unsigned long long> & dims)
     * @brief Generates 3d array of identity matrices of given dimensionss.
     *
     * @param[in] dims Dimensions of 3d array to create.
     * @return A 3d array of identity matrices.
     */
    static Matrix3d<Primitive, Key> createIdentityMatrices(const std::vector<unsigned long long> & dims,
                                                           const bool isRowMajor = Presets::DataSet::isRowMajor) {
        static std::string key{"ID"};

        auto primaryIdx{dims.size() > 2 ? dims[1] : dims[0]};
        auto secondaryIdx{dims.size() > 2 ? dims[2] : dims[1]};
        auto _range{dims.size() > 2 ? dims[0] : 1};

        Matrix3d<Primitive, Key> _metrics{};
        E_Matrix identity{E_Matrix::Identity(primaryIdx, secondaryIdx)};

        if (dims.size() > 2) {
            if (isRowMajor) {
                _metrics.addKey(key, (int) dims[0], (int) dims[1] * dims[2]);
                for (auto i = 0; i < _range; i++) {
                    _metrics().row(i) = E_Map(identity.data(), 1, dims[1] * dims[2]);
                }
            }
            else {
                _metrics.addKey(key, (int) dims[1] * dims[2], (int) dims[0]);
                for (auto i = 0; i < _range; i++) {
                    _metrics().col(i) = E_Map(identity.data(), dims[1] * dims[2], 1);
                }
            }
        } else {
            _metrics() = identity;
        }

        return _metrics;
    }

	bool dataToMatrix(void * source, const Key & key, std::vector<int> & dims, const bool isRowMajor = Presets::DataSet::isRowMajor) {
		bool result{};

        // Raw data buffer
        int _rows{};
        int _cols{};

        auto bufType{getType((int) dims.size())};

        // FIXME - buffer sizes need to be decided outside of this core function
        //      - overload!
        switch (bufType) {
            case DE::Set::Buffer::VECTOR:
                _rows = isRowMajor ? 1 : dims[0];
                _cols = isRowMajor ? dims[0] : 1;
                break;
            case DE::Set::Buffer::MATRIX_2d:
                _rows = isRowMajor ? dims[0] : dims[1];
                _cols = isRowMajor ? dims[1] : dims[0];
                break;
            case DE::Set::Buffer::MATRIX_3d:
                _rows = isRowMajor ? dims[0] * dims[1] : dims[0];
                _cols = isRowMajor ? dims[2] : dims[1] * dims[2];
                break;
        }

		if ((result = createBuffer(key, {(unsigned long long) _rows, (unsigned long long) _cols}, isRowMajor))) {
			mapMatrix(source, key);
		}	
		return result;
	}

    /**
     * @fn void mapMatrix(void * source, const Key & type, const bool isRowMajor = DataSet::isRowMajor)
     * @brief Map a raw data buffer into an Eigen::Matrix.
     *
     * Maps the raw data pointer into an Eigen::Matrix, then wraps the passed pointer in a ::Buffer. The memory
     *      management of said pointer is handled by the std::shared_ptr container.
     *
     * @param[in] source Raw data pointer to the beggining of the memory region containing the data to be mapped
     * @param[in] type dataset accessor
     * @param[in] isRowMajor storage-order of the raw data
     */
    void mapMatrix(void * source, const Key & type) {
        auto _source{static_cast<Primitive *>(source)};
        metrics[type] = E_Map(_source, metrics[type].rows(), metrics[type].cols());

        // Save raw buffer ptr (safely)
        (*dataBuffers[type]).data = source;
    }

    /**
     * @fn bool has(const Key & type)
     * @brief Check for dataset Key
     *
     * @param[in] type dataset accessor
     * @return Boolean expression whether the key is contained in Metrics<Primitive, Key>::metrics
     */
    bool has(const Key & type) {
        return metrics.find(type) != metrics.end();
    }

    /**
     * @fn std::vector<E_Matrix>& operator[](const Key & type)
     * @brief Access the 3d representation of the data.
     *
     * @param[in] type Dataset accessor
     * @return 3d Representation of data.
     */
    E_Matrix & operator[](const Key & type) {
        return metrics[type];
    }

    /**
     * @fn E_Matrix & operator()()
     * @brief Special dataset accessor operation.
     *
     * Should only be used if the Metrics obj stores only one set of matrices. (ie one key)
     *
     * @return Reference to the desired Eigen::Matrix.
     */
    E_Matrix & operator()() {
        return metrics[metrics.begin()->first];
    }

    /**
     * @fn std::shared_ptr<Buffer> getBuffer(const Key & type)
     * @brief Retrieve the raw data reference by dataset type.
     *
     * Note: Useful when converting 3d Eigen Matrices into cv::Mat()
     *
     * @param type dataset accessor
     * @return Shared pointer reference to the raw data.
     */
    std::shared_ptr<Buffer> getBuffer(const Key & type) {
        return dataBuffers[type];
    }

    unsigned long long getStride(const Key & key) {
        return dataBuffers[key]->stride;
    }

    /**
     * @fn std::vector<unsigned long long> getDimensions(const Key type)
     * @brief The dimensions of the data
     *
     * @param[in] type dataset accessor
     * @return Data dimensions in the format {matCount, rows, cols}
     */
    std::vector<unsigned long long> getDimensions(const Key type) {
        if (metrics.find(type) == metrics.end()) {
            std::cout << "Matrix has no dimensions." << std::endl;
            return {};
        }
        return dataBuffers[type]->dims;
    }

    /**
     * @fn bool createBuffer(const Key & type, const unsigned long long * dimensions, const int &rank, const bool isRowMajor = DataSet::isRowMajor)
     * @brief Allocate memory for data buffer and create corresponding empty Eigen matrices
     *
     * Note: uses dynamic memory allocation (new). Be mindful using in RT setting
     *
     * @param[in] type - dataset accessor
     * @param[in] dimensions - size of buffer to be created.
     * @param[in] rank - dimensiality of data
     * @param[in] isRowMajor storage-order of data
     * @return True - upon successful creation of buffer; False - otherwise
     */
    bool createBuffer(const Key & type, const unsigned long long * dimensions, const int &rank, const bool isRowMajor = Presets::DataSet::isRowMajor) {
        return createBuffer(type, Matrix3d<Primitive,Key>::squeeze(type, dimensions, rank), isRowMajor); // No need to reparse dimensiosn
    }

    /**
     * @fn bool createBuffer(const Key & type, const std::vector<unsigned long long> & dims, const bool isRowMajor = DataSet::isRowMajor)
     * @brief Allocate memory for data buffer and create corresponding empty Eigen matrices
     *
     * Note: uses dynamic memory allocation (new). Be mindful using in RT setting
     *
     * @param[in] key - dataset accessor
     * @param[in] dimensions - size of buffer to be created.
     * @param[in] isRowMajor storage-order of data
     * @return True - upon successful creation of buffer; False - otherwise
     */
    bool createBuffer(const Key & key, const std::vector<unsigned long long> & dims, const bool isRowMajor = Presets::DataSet::isRowMajor) {
        bool status{};
        // Check if ID already exists
        if ((status = (metrics.find(key) == metrics.end()))) {
            auto bufType{getType((int) dims.size())};

			//auto _rows{isRowMajor ? dims[0] * dims[1] : dims[1] * }
            auto stride{isMat3d(bufType) ? dims[1] * dims[2] : dims[1]};

//            auto buffer{std::shared_ptr<Buffer>{new Buffer{bufType, const_cast<std::vector<unsigned long long>&>(dims), stride}}};
			auto buffer{ std::shared_ptr<Buffer>{new Buffer{bufType, const_cast<std::vector<unsigned long long>&>(dims)}} };
            dataBuffers.insert({key, buffer});
            if (dataBuffers.find(key) == dataBuffers.end()) {
                std::cerr << "Failed to create raw data buffer : ID ( " << key << ")" << std::endl;
                return false;
            }

            // Eigen buffers
            metrics.insert({key, E_Matrix(dims[0], stride)});
//			metrics.insert({ key, E_Matrix(dims[0], stride) });
            if (metrics.find(key) == metrics.end()) {
                std::cerr << "Failed to create eigen matrix/vector : ID ( " << key << ")" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "Buffer already exists. Stopping overwrite" << std::endl;
            status = true;
        }
        return status;
    }

    /**
     * @fn void addKey(const Key & key, const int & numEigenMats)
     * @brief Add dataset key <b>without</b> allocating buffer.
     *
     * Create 3d representation of data, while only sizing the matrices' container. The Eigen matrices remain
     *      declared, but uninitialized.
     *
     * @param[in] key dataset accessor
     * @param[in] numEigenMats container size
     */
    void addKey(const Key & key, const int & rows, const int & cols, const bool flatten = false) { // , const int & numEigenMats
        metrics.insert({key, E_Matrix(rows, cols)});

        if (metrics.find(key) == metrics.end()) {
            std::cerr << "Failed to addKey to dataset : ID ( " << key << ")" << std::endl;
            return;
        }

        if (flatten) {
            metrics[key] = E_Map(metrics[key].data(), 1, rows * cols);
        }
    }

    void removeMat(const Key & key, const int & toRemove) {
        auto bufferLen{isRowMajor() ? metrics[key].rows() : metrics[key].cols()};

        assert(toRemove < bufferLen);

        auto rowLen{isRowMajor() ? metrics[key].rows() - 1 : metrics[key].rows()};
        auto colLen{isRowMajor() ? metrics[key].cols() : metrics[key].cols() - 1};

        auto mat3d{E_Matrix(rowLen, colLen)};

        // Mat3d shape after removal <= Mat to remove
        if (isRowMajor()) {
            auto upperRows{metrics[key].topRows(toRemove)};
            auto lowerRows{metrics[key].bottomRows(metrics[key].rows() - toRemove - 1)};
            mat3d << upperRows, lowerRows;
            metrics[key] = mat3d;
        }
        else {
            auto leftCols{metrics[key].leftCols(toRemove)};
            auto rightCols{metrics[key].rightCols(metrics[key].rows() - toRemove - 1)};
            mat3d << leftCols, rightCols;
            metrics[key] = mat3d;
        }

        isRowMajor() ? dataBuffers[key]->dims[0]-- : dataBuffers[key]->dims[1]--;
    }

    void info() {
        for (auto & pair : metrics) {
            auto dims{getDimensions(pair.first)};
            std::cout << "ID: " << pair.first << " ";
            switch (getType(dims.size())) {
                case DE::Set::Buffer::Type::VECTOR:
                    std::cout << "Dims: (" << dims[1] << ")" << std::endl;
                    break;
                case DE::Set::Buffer::Type::MATRIX_2d:
                    std::cout << "Dims: (" << dims[0] << ", " << dims[1] << ")" << std::endl;
                    break;
                case DE::Set::Buffer::Type::MATRIX_3d:
                    std::cout << "Dims: (" << dims[0] << ", " << dims[1] << ", " << dims[2] << ")" << std::endl;
                    break;
            }
        }
    }

    void writeToFile(const std::string & file, const Key & type) {
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream outStream{};
        outStream.open(file, std::ofstream::trunc);
        outStream << metrics[type];
        outStream.flush();
        outStream.close();
    }

    void writeToFiles(const std::string & file) {
        for (const auto & pair : metrics) {
            writeToFile(file + pair.first + ".csv", pair.first);
        }
    }

    /**
     * @fn std::vector<unsigned long long> squeeze(const Key & type, const unsigned long long * dimensions, const int & rank)
     * @brief Removes dimensions from dataset
     *
     * Similar to the python.numpy.squeeze() operation. Will remove the dimensiality of dataset with dimension less 1
     *
     * @param[in] type dataset accessor
     * @param[in] dimensions size of the buffer to be created
     * @param[in] rank dimensiality of data
     * @return
     */
    std::vector<unsigned long long> squeeze(const Key & type, const unsigned long long * dimensions, const int & rank) {
        std::vector<unsigned long long> DIMS{};
        for (auto i = 0; i < rank; i++) {
            // Similar to numpy.squeeze()
            if (dimensions[i] > 1) {
                DIMS.push_back(dimensions[i]);
            }
        }
        return DIMS;
    }

    /**
     * @fn  bool isRowMajor(const Key & type)
     * @brief Detemine storage-order of data.
     *
     * Developer's Note: The storage order of cv::Mat defaults to rowMajor, thus Eigen needs to be informed or
     *      all matrix operations need to be transposed. It was also found that while H5 file is read as rowMajor,
     *      the dataset DataSet::SetType::DEG (ie. "pan_tilt") is saved colMajor.
     *
     * @param type dataset accessor
     * @return True - if not DataSet::SetType::DEF (ie. "pan_tilt")
     */
    bool isRowMajor() {
      return storageOrder == Presets::DataSet::StorageOrder::ROW_MAJ;
    }

    std::string getStorageOrder() const {
        return storageOrder;
    }

    std::vector<std::pair<int, int>> indicesWithValuesAbove(const Key & key, const Primitive & threshold) {
        std::vector<std::pair<int, int>> indices{};
        Visit(metrics[key], [&indices, threshold](Primitive value, int i, int j) {
            if (value > threshold) {
                indices.emplace_back(i, j);
            }
        });

        return indices;
    }

    /**
     * @var std::unordered_map<Key, std::vector<E_Matrix>> metrics
     * @brief Map of 3d representation of data.
     *
     * Is the counter part to Metrics<Primitive, Key>::dataBuffers. High-level API which gives the user method of
     *      conducting linear algebra computations on the data, while not interacting with raw buffers.
     */
    std::unordered_map<Key, E_Matrix> metrics{};
};

#endif //METROLOGY2020_ALGORITHMS_METRICS_H