#ifndef QNN_RUNTIME_H
#define QNN_RUNTIME_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <dlfcn.h>

#include "QnnInterface.h"
#include "QnnGraph.h"
#include "QnnTensor.h"
#include "QnnTypes.h"
#include "QnnSystemInterface.h"



// Shape and quantization of a graph input/output.
// Dequantize with: real = (q + offset) * scale.
struct QnnTensorInfo {
    std::string name;
    std::vector<uint32_t> dims;
    float scale{1.0f};
    int32_t offset{0};
    size_t bytes{0};
};

class QnnRuntime {
public:
    QnnRuntime();
    ~QnnRuntime();

    bool initialize(
        const std::string& contextBinaryPath,
        const std::string& nativeLibDir
    );

    // Runs the graph. outputs[i] receives the raw (quantized)
    // bytes of output i, in the order of outputInfos().
    bool execute(
        const uint8_t* inputData,
        size_t inputSize,
        std::vector<std::vector<uint8_t>>& outputs
    );

    const QnnTensorInfo& inputInfo() const { return inputInfo_; }
    const std::vector<QnnTensorInfo>& outputInfos() const { return outputInfos_; }

    // Wall time of the last graphExecute() call.
    double lastExecuteMs() const { return lastExecuteMs_; }

    void shutdown();

    bool isInitialized() const;

    const std::string& getLastError() const;

private:
    bool loadQnnLibraries();
    bool loadQnnInterface();
    bool loadQnnSystemInterface();

    bool initializeBackend();
    bool createContextFromBinary();
    bool retrieveGraph();

    bool setupInputTensor();
    bool setupOutputTensor();

    bool readContextBinary(std::vector<uint8_t>& binary);

    void cleanupModel();
    void cleanupContext();
    void cleanupBackend();
    void closeLibraries();

    bool setError(const std::string& error);

private:
    QNN_INTERFACE_VER_TYPE qnnInterface_{};
    QNN_SYSTEM_INTERFACE_VER_TYPE qnnSystemInterface_{};

    Qnn_LogHandle_t logHandle_{nullptr};
    Qnn_BackendHandle_t backendHandle_{nullptr};
    Qnn_DeviceHandle_t deviceHandle_{nullptr};
    Qnn_ContextHandle_t contextHandle_{nullptr};
    Qnn_GraphHandle_t graphHandle_{nullptr};
    std::string graphName_;
    Qnn_ProfileHandle_t profileHandle_{nullptr};

    void* qnnHtpLib_{nullptr};
    void* qnnHtpPrepareLib_{nullptr};
    void* qnnSystemLib_{nullptr};

    using QnnInterfaceGetProvidersFn =
        Qnn_ErrorHandle_t (*)(
            const QnnInterface_t*** providers,
            uint32_t* numProviders
        );

    using QnnSystemInterfaceGetProvidersFn =
        Qnn_ErrorHandle_t (*)(
            const QnnSystemInterface_t*** providers,
            uint32_t* numProviders
        );

    QnnInterfaceGetProvidersFn qnnInterfaceGetProviders_{nullptr};
    QnnSystemInterfaceGetProvidersFn qnnSystemInterfaceGetProviders_{nullptr};

    Qnn_Tensor_t inputTensor_{};
    std::vector<Qnn_Tensor_t> outputTensors_;

    QnnTensorInfo inputInfo_;
    std::vector<QnnTensorInfo> outputInfos_;
    double lastExecuteMs_{0.0};

    std::string contextBinaryPath_;

    bool qnnLoaded_{false};
    bool backendInitialized_{false};
    bool contextInitialized_{false};
    bool graphInitialized_{false};

    std::string lastError_;
};

#endif  // QNN_RUNTIME_H
