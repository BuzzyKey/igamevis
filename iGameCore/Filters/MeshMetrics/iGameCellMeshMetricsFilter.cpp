#include "iGameCellMeshMetricsFilter.h"
#include "iGameSurfaceMeshMetricsFilter.h"

IGAME_NAMESPACE_BEGIN

CellMeshMetricsFilter::CellMeshMetricsFilter() {
    this->SetNumberOfInputs(1);
    this->SetNumberOfOutputs(1);
    input = nullptr;
    output = nullptr;
}

bool CellMeshMetricsFilter::Execute() {
    if (m_Inputs->GetNumberOfElements() == 0) { return false; }
    input = m_Inputs->GetElement(0);
    return Execute(input);
}

bool CellMeshMetricsFilter::Execute(DataObject::Pointer input) {
    if (!input) { return false; }
    this->output = DataObject::New();
    return Execute(input, output);
}

bool CellMeshMetricsFilter::Execute(DataObject::Pointer input, DataObject::Pointer& output) {
    if (!input) { return false; }
    m_SupportedCount = 0;
    m_UnsupportedCount = 0;
    m_SkippedBlockCount = 0;
    bool success = ComputeCellMetrics(input, output);
    if (success && output) {
        this->SetOutput(0, output);
        return true;
    }
    return false;
}

bool CellMeshMetricsFilter::ComputeCellMetrics(DataObject::Pointer input, DataObject::Pointer& output) {
    if (!input) { return false; }

    // Branch node: recursive traversal
    if (input->HasSubDataObject()) {
        // 这里必须用 DrawObject：复合容器传入 Scene 时会被强转成 DrawObject
        DrawObject::Pointer outputContainer = DrawObject::New();
        outputContainer->SetName(input->GetName()); // Preserve name hierarchy

        for (auto it = input->SubDataObjectIteratorBegin(); it != input->SubDataObjectIteratorEnd(); ++it) {
            DataObject::Pointer subOutput;
            if (ComputeCellMetrics(it->second, subOutput)) {
                if (subOutput) { outputContainer->AddSubDataObject(subOutput); }
            } else {
                // Todo::没成功计算怎么办？保留网格，不能静默删除
                if (it->second) { outputContainer->AddSubDataObject(it->second); }
            }
        }
        output = outputContainer;
        return true;
    }
    // Leaf node: compute volume cell quality metrics
    else {
        auto srcMesh = DynamicCast<UnstructuredMesh>(input);
        if (!srcMesh || srcMesh->GetNumberOfCells() == 0) {
            // 块级跳过：不是可处理的非结构网格（可能是 VolumeMesh / SurfaceMesh / 空网格）
            // 原样保留，不静默丢弃；同时计数并在日志中报告块名
            m_SkippedBlockCount++;
            output = input;
            IGAME_CORE_WARN("[CellMeshMetrics] Block skipped: not an UnstructuredMesh or NumberOfCells == 0, name='{}'",
                            input->GetName());
            return true;
        }

        // 1. 创建新网格
        auto outMesh = UnstructuredMesh::New();
        outMesh->SetName(srcMesh->GetName());
        outMesh->SetPoints(srcMesh->GetPoints());                         // 共享只读几何点
        outMesh->SetCells(srcMesh->GetCells(), srcMesh->GetCellTypes());  // 共享拓扑和权威类型
        // 此处浅拷贝函数有问题，未实现
        // outMesh->GetAttributeSet()->ShallowCopy(srcMesh->GetAttributeSet()); // 浅拷贝已有属性
        auto attrSet = srcMesh->GetAttributeSet();
        auto attrAll = attrSet->GetAllAttributes();
        for (auto i = attrAll->Begin(); i < attrAll->End(); i++) { 
            outMesh->GetAttributeSet()->AddAttribute(i->type, i->attachmentType, i->GetPointer());
        }

        IGsize numCells = srcMesh->GetNumberOfCells();
        auto qualityArray = DoubleArray::New();
        qualityArray->SetName("CellQuality");
        qualityArray->Resize(numCells);

        igIndex vhs[IGAME_CELL_MAX_SIZE] = {0};

        // 1.5 预扫描：Relative Size Squared / Shape and Size / Shear and Size
        //     这三个指标需要【全网格平均单元尺寸】作为额外输入。
        //     ParaView 对每种单元类型分别取该类型的平均值（面积或体积），
        //     因此这里按类型各算一份，逐单元分发时取对应类型的那份。
        double avgTetVolume = 0.0, avgHexVolume = 0.0;
        double avgQuadArea = 0.0, avgTriArea = 0.0;
        if (m_QualityMetric == QUALITY_RELATIVE_SIZE_SQUARED ||
            m_QualityMetric == QUALITY_SHAPE_AND_SIZE ||
            m_QualityMetric == QUALITY_SHEAR_AND_SIZE) {
            double tetSum = 0.0, hexSum = 0.0, quadSum = 0.0, triSum = 0.0;
            int tetN = 0, hexN = 0, quadN = 0, triN = 0;
            for (IGsize c = 0; c < numCells; c++) {
                IGCellType t = static_cast<IGCellType>(srcMesh->GetCellType(c));
                const int vn = srcMesh->GetCellPointIds(c, vhs);
                std::vector<Point> sp;
                sp.reserve(vn);
                for (int i = 0; i < vn; i++) { sp.push_back(srcMesh->GetPoints()->GetPoint(vhs[i])); }

                if (t == IG_TETRA && vn == 4) {
                    // 注意：这里用【有符号】体积（Verdict tet_volume 的口径），
                    // 不能用 ComputeTetVolume（它取绝对值，会改变平均值）。
                    const double ax = static_cast<double>(sp[1][0]) - sp[0][0];
                    const double ay = static_cast<double>(sp[1][1]) - sp[0][1];
                    const double az = static_cast<double>(sp[1][2]) - sp[0][2];
                    const double bx = static_cast<double>(sp[2][0]) - sp[0][0];
                    const double by = static_cast<double>(sp[2][1]) - sp[0][1];
                    const double bz = static_cast<double>(sp[2][2]) - sp[0][2];
                    const double cx = static_cast<double>(sp[3][0]) - sp[0][0];
                    const double cy = static_cast<double>(sp[3][1]) - sp[0][1];
                    const double cz = static_cast<double>(sp[3][2]) - sp[0][2];
                    const double crx = ay * bz - az * by;
                    const double cry = az * bx - ax * bz;
                    const double crz = ax * by - ay * bx;
                    tetSum += (cx * crx + cy * cry + cz * crz) / 6.0;
                    tetN++;
                } else if (t == IG_HEXAHEDRON && vn == 8) {
                    hexSum += VolumeMeshMetricsFilter::ComputeHexVolume(sp); hexN++;
                } else if (t == IG_QUAD && vn == 4) {
                    quadSum += SurfaceMeshMetricsFilter::ComputeQuadArea(sp); quadN++;
                } else if (t == IG_TRIANGLE && vn == 3) {
                    triSum += SurfaceMeshMetricsFilter::ComputeTriangleArea(sp); triN++;
                }
            }
            if (tetN > 0) { avgTetVolume = tetSum / tetN; }
            if (hexN > 0) { avgHexVolume = hexSum / hexN; }
            if (quadN > 0) { avgQuadArea = quadSum / quadN; }
            if (triN > 0) { avgTriArea = triSum / triN; }
        }

        // 2. 每个Cell进行计算
        for (IGsize c = 0; c < numCells; c++) {
            IGCellType type = static_cast<IGCellType>(srcMesh->GetCellType(c));
            int vNum = srcMesh->GetCellPointIds(c, vhs);

            // 单元点集
            std::vector<Point> pts;
            pts.reserve(vNum);
            for (int i = 0; i < vNum; i++) {
                pts.push_back(srcMesh->GetPoints()->GetPoint(vhs[i]));
            }
            
            double val = m_UnsupportedValue;
            bool supported = false;
            // 根据真实单元类型，直接调用 VolumeMeshMetricsFilter 的静态数学函数！
            if (type == IG_TETRA && vNum == 4) {
                switch (m_QualityMetric) {
                    case QUALITY_EDGE_RATIO:
                        val = VolumeMeshMetricsFilter::ComputeTetEdgeRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_VOLUME:
                        val = VolumeMeshMetricsFilter::ComputeTetVolume(pts);
                        supported = true;
                        break;
                    case QUALITY_ASPECT_RATIO:
                        val = VolumeMeshMetricsFilter::GetAspectRatioOfCell(pts);
                        supported = true;
                        break;
                    case QUALITY_JACOBIAN:
                        val = VolumeMeshMetricsFilter::GetJacobianOfCell(pts);
                        supported = true;
                        break;
                    case QUALITY_MIN_ANGLE:
                        val = VolumeMeshMetricsFilter::GetMinDihedralAngleOfTet(pts); // ← 原来 GetMinInternalAnglesOfCell
                        supported = true;
                        break;
                    case QUALITY_COLLAPSE_RATIO:
                        val = VolumeMeshMetricsFilter::GetCollapseRatioOfCell(pts);
                        supported = true;
                        break;
                    // —— 对齐 ParaView Cell Quality 新增 ——
                    case QUALITY_CONDITION:
                        val = VolumeMeshMetricsFilter::ComputeTetCondition(pts);
                        supported = true;
                        break;
                    case QUALITY_DISTORTION:
                        val = VolumeMeshMetricsFilter::ComputeTetDistortion(pts);
                        supported = true;
                        break;
                    case QUALITY_EQUIANGLE_SKEW:
                        val = VolumeMeshMetricsFilter::ComputeTetEquiangleSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_RADIUS_RATIO:
                        val = VolumeMeshMetricsFilter::ComputeTetRadiusRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_SCALED_JACOBIAN:
                        val = VolumeMeshMetricsFilter::ComputeTetScaledJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_SHAPE:
                        val = VolumeMeshMetricsFilter::ComputeTetShape(pts);
                        supported = true;
                        break;
                    // —— 需要全网格平均单元体积的 3 项 ——
                    case QUALITY_RELATIVE_SIZE_SQUARED:
                        if (avgTetVolume > 0.0) {
                            val = VolumeMeshMetricsFilter::ComputeTetRelativeSizeSquared(pts, avgTetVolume);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHAPE_AND_SIZE:
                        if (avgTetVolume > 0.0) {
                            val = VolumeMeshMetricsFilter::ComputeTetShapeAndSize(pts, avgTetVolume);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHEAR_AND_SIZE:
                        // 四面体没有 shear_and_size（ParaView 同口径）
                        break;
                    default:
                        break; // 该指标对四面体不适用（对齐 ParaView）
                }
            } else if (type == IG_HEXAHEDRON && vNum == 8) {
                switch (m_QualityMetric) {
                    case QUALITY_EDGE_RATIO:
                        val = VolumeMeshMetricsFilter::ComputeHexEdgeRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_VOLUME:
                        val = VolumeMeshMetricsFilter::ComputeHexVolume(pts);
                        supported = true;
                        break;
                    case QUALITY_JACOBIAN:
                        val = VolumeMeshMetricsFilter::ComputeHexJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_SKEW:
                        val = VolumeMeshMetricsFilter::ComputeHexSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_TAPER:
                        val = VolumeMeshMetricsFilter::ComputeHexTaper(pts);
                        supported = true;
                        break;
                    case QUALITY_STRETCH:
                        val = VolumeMeshMetricsFilter::ComputeHexStretch(pts);
                        supported = true;
                        break;
                    case QUALITY_DIAGONAL:
                        val = VolumeMeshMetricsFilter::ComputeHexDiagonal(pts);
                        supported = true;
                        break;
                    case QUALITY_MAX_EDGE_RATIO:
                        val = VolumeMeshMetricsFilter::ComputeHexMaxEdgeRatio(pts);
                        supported = true;
                        break;
                    // —— 对齐 ParaView Cell Quality 新增 ——
                    case QUALITY_CONDITION:
                        val = VolumeMeshMetricsFilter::ComputeHexCondition(pts);
                        supported = true;
                        break;
                    case QUALITY_DISTORTION:
                        val = VolumeMeshMetricsFilter::ComputeHexDistortion(pts);
                        supported = true;
                        break;
                    case QUALITY_EQUIANGLE_SKEW:
                        val = VolumeMeshMetricsFilter::ComputeHexEquiangleSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_ODDY:
                        val = VolumeMeshMetricsFilter::ComputeHexOddy(pts);
                        supported = true;
                        break;
                    case QUALITY_SCALED_JACOBIAN:
                        val = VolumeMeshMetricsFilter::ComputeHexScaledJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_SHAPE:
                        val = VolumeMeshMetricsFilter::ComputeHexShape(pts);
                        supported = true;
                        break;
                    case QUALITY_SHEAR:
                        val = VolumeMeshMetricsFilter::ComputeHexShear(pts);
                        supported = true;
                        break;
                    // —— 需要全网格平均单元体积的 3 项 ——
                    case QUALITY_RELATIVE_SIZE_SQUARED:
                        if (avgHexVolume > 0.0) {
                            val = VolumeMeshMetricsFilter::ComputeHexRelativeSizeSquared(
                                    pts, avgHexVolume);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHAPE_AND_SIZE:
                        if (avgHexVolume > 0.0) {
                            val = VolumeMeshMetricsFilter::ComputeHexShapeAndSize(
                                    pts, avgHexVolume);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHEAR_AND_SIZE:
                        if (avgHexVolume > 0.0) {
                            val = VolumeMeshMetricsFilter::ComputeHexShearAndSize(
                                    pts, avgHexVolume);
                            supported = true;
                        }
                        break;
                    default:
                        break; // 该指标对六面体不适用（对齐 ParaView）
                }
            } else if (type == IG_TRIANGLE && vNum == 3) {
                // 三角形适用的指标（对齐 ParaView Cell Quality 实测矩阵）：
                //   Area / Aspect Ratio / Condition / Distortion / Edge Ratio /
                //   Equiangle Skew / Minimum Angle / Radius Ratio /
                //   Scaled Jacobian / Shape  —— 共 10 项
                switch (m_QualityMetric) {
                    case QUALITY_AREA:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleArea(pts);
                        supported = true;
                        break;
                    case QUALITY_ASPECT_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleAspectRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_CONDITION:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleCondition(pts);
                        supported = true;
                        break;
                    case QUALITY_DISTORTION:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleDistortion(pts);
                        supported = true;
                        break;
                    case QUALITY_EDGE_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleEdgeRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_EQUIANGLE_SKEW:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleEquiangleSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_MIN_ANGLE:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleMinAngle(pts);
                        supported = true;
                        break;
                    case QUALITY_RADIUS_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleRadiusRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_SCALED_JACOBIAN:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleScaledJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_SHAPE:
                        val = SurfaceMeshMetricsFilter::ComputeTriangleShape(pts);
                        supported = true;
                        break;
                    // —— 需要全网格平均面积的 2 项 ——
                    case QUALITY_RELATIVE_SIZE_SQUARED:
                        if (avgTriArea > 0.0) {
                            val = SurfaceMeshMetricsFilter::ComputeTriangleRelativeSizeSquared(
                                    pts, avgTriArea);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHAPE_AND_SIZE:
                        if (avgTriArea > 0.0) {
                            val = SurfaceMeshMetricsFilter::ComputeTriangleShapeAndSize(
                                    pts, avgTriArea);
                            supported = true;
                        }
                        break;
                    default:
                        break; // 其余指标对三角形不适用（ParaView 同口径）
                }
            } else if (type == IG_QUAD && vNum == 4) {
                // 四边形适用的指标（对齐 ParaView Cell Quality 实测矩阵）：
                //   Area / Aspect Ratio / Condition / Distortion / Edge Ratio /
                //   Equiangle Skew / Jacobian / Minimum Angle / Oddy /
                //   Radius Ratio / Scaled Jacobian / Shape / Shear /
                //   Skew / Stretch / Taper / Warpage —— 共 17 项
                switch (m_QualityMetric) {
                    case QUALITY_AREA:
                        val = SurfaceMeshMetricsFilter::ComputeQuadArea(pts);
                        supported = true;
                        break;
                    case QUALITY_ASPECT_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeQuadAspectRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_CONDITION:
                        val = SurfaceMeshMetricsFilter::ComputeQuadCondition(pts);
                        supported = true;
                        break;
                    case QUALITY_DISTORTION:
                        val = SurfaceMeshMetricsFilter::ComputeQuadDistortion(pts);
                        supported = true;
                        break;
                    case QUALITY_EDGE_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeQuadEdgeRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_EQUIANGLE_SKEW:
                        val = SurfaceMeshMetricsFilter::ComputeQuadEquiangleSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_JACOBIAN:
                        val = SurfaceMeshMetricsFilter::ComputeQuadJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_MIN_ANGLE:
                        val = SurfaceMeshMetricsFilter::ComputeQuadMinAngle(pts);
                        supported = true;
                        break;
                    case QUALITY_ODDY:
                        val = SurfaceMeshMetricsFilter::ComputeQuadOddy(pts);
                        supported = true;
                        break;
                    case QUALITY_RADIUS_RATIO:
                        val = SurfaceMeshMetricsFilter::ComputeQuadRadiusRatio(pts);
                        supported = true;
                        break;
                    case QUALITY_SCALED_JACOBIAN:
                        val = SurfaceMeshMetricsFilter::ComputeQuadScaledJacobian(pts);
                        supported = true;
                        break;
                    case QUALITY_SHAPE:
                        val = SurfaceMeshMetricsFilter::ComputeQuadShape(pts);
                        supported = true;
                        break;
                    case QUALITY_SHEAR:
                        val = SurfaceMeshMetricsFilter::ComputeQuadShear(pts);
                        supported = true;
                        break;
                    case QUALITY_SKEW:
                        val = SurfaceMeshMetricsFilter::ComputeQuadSkew(pts);
                        supported = true;
                        break;
                    case QUALITY_TAPER:
                        val = SurfaceMeshMetricsFilter::ComputeQuadTaper(pts);
                        supported = true;
                        break;
                    case QUALITY_STRETCH:
                        val = SurfaceMeshMetricsFilter::ComputeQuadStretch(pts);
                        supported = true;
                        break;
                    case QUALITY_WARPAGE:
                        val = SurfaceMeshMetricsFilter::ComputeQuadWarpage(pts);
                        supported = true;
                        break;
                    // —— 需要全网格平均面积的 3 项 ——
                    case QUALITY_RELATIVE_SIZE_SQUARED:
                        if (avgQuadArea > 0.0) {
                            val = SurfaceMeshMetricsFilter::ComputeQuadRelativeSizeSquared(
                                    pts, avgQuadArea);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHAPE_AND_SIZE:
                        if (avgQuadArea > 0.0) {
                            val = SurfaceMeshMetricsFilter::ComputeQuadShapeAndSize(
                                    pts, avgQuadArea);
                            supported = true;
                        }
                        break;
                    case QUALITY_SHEAR_AND_SIZE:
                        if (avgQuadArea > 0.0) {
                            val = SurfaceMeshMetricsFilter::ComputeQuadShearAndSize(
                                    pts, avgQuadArea);
                            supported = true;
                        }
                        break;
                    default:
                        break; // 有限差分类指标对四边形不适用（ParaView 同口径）
                }
            }

            if (supported) {
                qualityArray->SetValue(c, val);
                m_SupportedCount++;
            } else {
                qualityArray->SetValue(c, m_UnsupportedValue);
                m_UnsupportedCount++;
            }
        }
        
        // 3. 加入新网格
        outMesh->GetAttributeSet()->AddAttribute(IG_SCALAR, IG_CELL, qualityArray);
        output = outMesh;
        return true;

    }
}

IGAME_NAMESPACE_END
