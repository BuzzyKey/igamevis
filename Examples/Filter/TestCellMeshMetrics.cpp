// ===========================================================================
// Cell Quality 批量导出 / 与 ParaView 基准逐值对比
// ---------------------------------------------------------------------------
// 【背景 BUG】CellMeshMetricsFilter 最初只处理四面体与六面体，遇到三角形 /
//   四边形单元时不做任何分支，导致全部单元落到「不支持值」：
//     - 现象：对 Convert_Quad_Bicycle.vtk（65896 四边形 + 30772 三角形）执行
//             Edge Ratio，报告「成功 0 / 跳过 96668」，而 ParaView 同一文件
//             能正常算出 [1.0000979, 483.93376]。
//     - 触发条件：模型含面单元（三角形或四边形），任一指标。
//     - 影响：Cell Quality 与 ParaView 结果不可比，用户无法判断是模型不适用
//             还是功能缺失。
//
// 【本测试要保证的正确行为】
//   1. 对三角形单元至少支持 Edge Ratio / Aspect Ratio / Minimum Angle；
//      其余 4 个指标（Jacobian / Skew / Taper / Stretch）应填「不支持值」。
//   2. 对四边形单元支持 7 个指标中的 6 个（Stretch 待实现），
//      且 成功数 + 跳过数 == 模型总单元数。
//   3. 每个指标的数值应与 ParaView 6.1.1 的 vtkCellQuality 逐单元一致。
//
// 【重要边界用例及原因】
//   - 混合网格（四边形 + 三角形）必须逐类型分发：同一指标在不同类型上的
//     支持性不同，若只按「第一个单元的类型」推断会给错结果。
//   - 「不支持值」使用 -1.0（对齐 ParaView 的 Undefined Quality 默认值），
//     统计范围时必须排除，否则会污染 min/max。
//
// 【修复提交】待提交。测试与修复同次首次提交，提交后用
//   git log --diff-filter=A --format="%h %s" -- Examples/Filter/TestCellMeshMetrics.cpp
//   查询并回填准确提交号。
//
// 【用法】testCellMeshMetrics [<模型路径>] [<输出目录>]
//   默认模型 = Convert_Quad_Bicycle.vtk，默认输出 = ./cellquality-out
//   对 7 个统一指标逐个评估并导出逐单元 CSV；CSV 格式与 ParaView 导出的
//   基准完全一致，可直接交给 compare_cellquality.py 做逐值比对。
// ===========================================================================

#include <Core/iGameScene.h>
#include <MeshMetrics/iGameCellMeshMetricsFilter.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <iGameDrawObject.h>
#include <iGameFileIO.h>
#include <iGameInteractor.h>
#include <iGameRenderWindow.h>
#include <iGameUnstructuredMesh.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

using CellQualityMetric = iGame::CellMeshMetricsFilter::CellQualityMetric;

// 智能模型路径查找（无论在根目录还是构建目录都能自动找到）
std::string FindModelPath(const std::string& modelName) {
    std::vector<std::string> searchPaths = {modelName,
                                            "./" + modelName,
                                            "./Examples/Models/" + modelName,
                                            "../Examples/Models/" + modelName,
                                            "../../Examples/Models/" + modelName,
                                            "../../../Examples/Models/" + modelName};
    for (const auto& path: searchPaths) {
        if (std::filesystem::exists(path)) { return path; }
    }
    return modelName;
}

// 7 个统一指标（与 UI 下拉框、ParaView Cell Quality 可选值保持一致）
struct MetricCase {
    const char* label;    // 中文名 (English)
    const char* paraName; // ParaView 下拉框名称，同时用作 CSV 文件名
    CellQualityMetric metric;
};

const std::vector<MetricCase> kMetricCases = {
        {"边长比 (Edge Ratio)", "Edge Ratio", CellQualityMetric::QUALITY_EDGE_RATIO},
        {"纵横比 (Aspect Ratio)", "Aspect Ratio", CellQualityMetric::QUALITY_ASPECT_RATIO},
        {"雅可比行列式 (Jacobian)", "Jacobian", CellQualityMetric::QUALITY_JACOBIAN},
        {"歪斜度 (Skew)", "Skew", CellQualityMetric::QUALITY_SKEW},
        {"最小内角 (Minimum Angle)", "Minimum Angle", CellQualityMetric::QUALITY_MIN_ANGLE},
        {"锥度 (Taper)", "Taper", CellQualityMetric::QUALITY_TAPER},
        {"伸展度 (Stretch)", "Stretch", CellQualityMetric::QUALITY_STRETCH},
};

// 递归收集叶子节点的 CellQuality 数组（按遍历顺序拼接）
void CollectQuality(iGame::DataObject::Pointer node, std::vector<double>& out) {
    if (!node) { return; }
    if (node->HasSubDataObject()) {
        for (auto it = node->SubDataObjectIteratorBegin(); it != node->SubDataObjectIteratorEnd(); ++it) {
            CollectQuality(it->second, out);
        }
        return;
    }
    auto attrSet = node->GetAttributeSet();
    if (!attrSet) { return; }
    for (int i = 0; i < (int) attrSet->GetNumberOfAttributes(); ++i) {
        auto& attr = attrSet->GetAttribute(i);
        if (attr.isDeleted || !attr.pointer) { continue; }
        if (attr.pointer->GetName() != "CellQuality") { continue; }
        for (IGsize c = 0; c < (IGsize) attr.pointer->GetNumberOfElements(); ++c) {
            out.push_back(attr.pointer->GetValue(c));
        }
        break;
    }
}

// 把 ParaView 名称转成文件名安全形式（空格 -> 下划线）
std::string SafeName(const std::string& s) {
    std::string r = s;
    for (char& c: r) {
        if (c == ' ' || c == '/' || c == '\\') { c = '_'; }
    }
    return r;
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(65001); // Windows 控制台 UTF-8 支持
#endif
    std::cout << std::unitbuf;

    const std::string modelArg = (argc > 1) ? argv[1] : "Convert_Quad_Bicycle.vtk";
    const std::string outDirArg = (argc > 2) ? argv[2] : "./cellquality-out";

    std::cout << "\n==============================================================================\n";
    std::cout << "  【iGameVis】Cell Quality 批量导出 —— 供与 ParaView 基准逐值对比\n";
    std::cout << "==============================================================================\n";

    const std::string modelPath = FindModelPath(modelArg);
    std::cout << "\n[步骤 1] 读取模型: " << modelPath << "\n";
    auto root = iGame::FileIO::ReadFile(modelPath);
    if (!root) {
        std::cerr << "  [错误] 读取模型失败: " << modelPath << "\n";
        return -1;
    }
    std::cout << "  读取成功\n";

    std::filesystem::create_directories(outDirArg);
    std::cout << "  输出目录: " << std::filesystem::absolute(outDirArg).string() << "\n";

    const double kUnsupportedValue = -1.0;
    const std::string baseName = std::filesystem::path(modelPath).stem().string();

    std::cout << "\n[步骤 2] 逐个指标评估并导出 CSV...\n";
    const std::string line(84, '-');
    std::cout << line << "\n";
    std::cout << std::left << std::setw(28) << "  指标"
              << std::setw(12) << "成功"
              << std::setw(12) << "跳过"
              << "范围\n";
    std::cout << line << "\n";

    int failCount = 0;
    for (const auto& mc: kMetricCases) {
        auto filter = iGame::CellMeshMetricsFilter::New();
        filter->setMetric(mc.metric);
        filter->setUnsupportedValue(kUnsupportedValue);
        filter->SetInput(0, root);
        if (!filter->Execute()) {
            std::cout << "  " << std::left << std::setw(26) << mc.label << "  [FAIL] 执行失败\n";
            ++failCount;
            continue;
        }
        auto outputObj = filter->GetOutput(0);
        if (!outputObj) {
            std::cout << "  " << std::left << std::setw(26) << mc.label << "  [FAIL] 输出为空\n";
            ++failCount;
            continue;
        }

        std::vector<double> values;
        CollectQuality(outputObj, values);

        double vmin = DBL_MAX, vmax = -DBL_MAX;
        for (double v: values) {
            if (v == kUnsupportedValue) { continue; } // 不支持值不计入范围
            vmin = std::min(vmin, v);
            vmax = std::max(vmax, v);
        }
        if (values.empty() || vmin > vmax) { vmin = vmax = 0.0; }

        // 导出 CSV（格式与 ParaView 基准一致）
        const std::string csvPath =
                (std::filesystem::path(outDirArg) / (baseName + "__" + SafeName(mc.paraName) + ".csv")).string();
        std::ofstream ofs(csvPath);
        ofs << "# model=" << baseName << ",measure=" << mc.paraName
            << ",min=" << std::setprecision(17) << vmin
            << ",max=" << vmax
            << ",cells=" << values.size() << "\n";
        ofs << "cell_index,CellQuality\n";
        for (size_t i = 0; i < values.size(); ++i) {
            ofs << i << "," << std::setprecision(17) << values[i] << "\n";
        }
        ofs.close();

        std::cout << "  " << std::left << std::setw(26) << mc.label
                  << std::setw(12) << filter->GetSupportedCount()
                  << std::setw(12) << filter->GetUnsupportedCount()
                  << "[" << std::setprecision(8) << vmin << ", " << vmax << "]\n";
    }

    std::cout << line << "\n";
    std::cout << "  完成: " << (kMetricCases.size() - failCount) << " / " << kMetricCases.size()
              << " 个指标已导出到 " << outDirArg << "\n";
    std::cout << "==============================================================================\n";

    return failCount == 0 ? 0 : 1;
}
