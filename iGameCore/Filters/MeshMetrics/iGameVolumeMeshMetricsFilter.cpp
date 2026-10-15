#include "iGameVolumeMeshMetricsFilter.h"
#include "Convert/iGameConvertToVolumeMeshFilter.h"


IGAME_NAMESPACE_BEGIN

VolumeMeshMetricsFilter::VolumeMeshMetricsFilter() {
    this->SetNumberOfInputs(1);
    this->SetNumberOfOutputs(1);
    m_Cells = nullptr;
    m_Points = nullptr;
}

VolumeMeshMetricsFilter::~VolumeMeshMetricsFilter() {
    m_Cells = nullptr;
    m_Points = nullptr;
}


bool VolumeMeshMetricsFilter::Execute() {
    if (m_Inputs->GetNumberOfElements() == 0) { return false; }

    auto input = m_Inputs->GetElement(0);
    if (!input) { return false; }
    m_Cells = nullptr;
    switch (input->GetDataObjectType()) {
        case IG_VOLUME_MESH:
            m_Cells = (DynamicCast<VolumeMesh>(input))->GetCells();
            m_Points = (DynamicCast<VolumeMesh>(input))->GetPoints();
            break;
        case IG_UNSTRUCTURED_MESH: {
            auto mesh = DynamicCast<UnstructuredMesh>(input);
            auto converter = ConvertToVolumeMeshFilter::New();
            converter->SetInput(mesh);
            bool result = converter->Execute();
            if (result) {
                m_Cells = converter->GetVolumeMesh()->GetCells();
                m_Points = converter->GetVolumeMesh()->GetPoints();
            }
            break;
        }
        default:
            igDebug("请输入体网格进行质量检测");
            break;
    }
    if (!m_Cells) {
        igDebug("没有体网格单元");
        return false;
    }
    igIndex vhs[IGAME_CELL_MAX_SIZE] = {0}; //存储每个面的顶点索引数组
    igIndex vNum = 0;                       //每个面的顶点数量

    igIndex cellNum = m_Cells->GetNumberOfCells(); //总面数

    DoubleArray::Pointer metricArray = DoubleArray::New();
    metricArray->SetName("Metric" + std::to_string(m_Metric));

    metricArray->SetDimension(1);
    metricArray->Reserve(cellNum);
    DoubleArray::Pointer dataRange = DoubleArray::New();
    
    dataRange->AddValue(0.0);
    dataRange->AddValue(1.0);
    dataRange->AddValue(0.0);
    dataRange->AddValue(1.0);
    for (igIndex i = 0; i < cellNum; i++) {
        vNum = m_Cells->GetCellIds(i, vhs);             // 获取顶点索引
        double metric = this->ComputeMetric(vNum, vhs); // 计算质量指标
        metricArray->AddValue(metric);                  // 存储结果
    }
    
    input->GetAttributeSet()->AddAttribute(IG_SCALAR, IG_CELL, metricArray);
    this->SetOutput(input);
    return true;
}


double VolumeMeshMetricsFilter::ComputeMetric(igIndex vNum, igIndex* vhs) {


    std::vector<Point> points;
    for (igIndex i = 0; i < vNum; i++) { points.push_back(m_Points->GetPoint(vhs[i])); }
    //Point p[100];
    //for (igIndex i = 0; i < vNum; i++) {
    //	p[i]=m_Points->GetPoint(vhs[0]);
    //}
    //double len= (p[1]-p[0]).norm();
    //return ComputeA(vNum, vhs);

    // 根据顶点数量选择计算方法
    if (vNum == 4) {
        // 四面体
        switch (m_Metric) {
            case TET_EDGE_RATIO:
                return ComputeTetEdgeRatio(points);
                break;
            case TET_VOLUME:
                return ComputeTetVolume(points);
                break;
            case TET_ASPECT_RATIO:
                return GetAspectRatioOfCell(points);
                break;
            case TET_JACOBIAN:
                return GetJacobianOfCell(points);
                break;
            case TET_COLLAPSE_RATIO:
                return GetCollapseRatioOfCell(points);
                break;
            case TET_VOL_SKEW:
                return GetVolSkewOfCell(points);
                break;
            case TET_MIN_ANGLE:
                return GetMinInternalAnglesOfCell(points);
                break;
            case TET_EQUIANGLE_SKEWNESS:
                return GetEquiangleSkewnessOfCell(points);
                break;
            case TET_INRADIUS:
                return GetInradiusOfCell(points);
                break;
            case TET_CIRCUMRADIUS:
                return GetCircumradiusOfCell(points);
                break;
            case TET_VOL_ASPECT_RATIO:
                return GetVolAspectRatioOfCell(points);
                break;
            default:
                break;
        }
    } else if (vNum == 8) {
        // 六面体
        switch (m_Metric) {
            case HEX_VOLUME:
                return ComputeHexVolume(points);
                break;
            case HEX_TAPER:
                return ComputeHexTaper(points);
                break;
            case HEX_JACOBIAN:
                return ComputeHexJacobian(points);
                break;
            case HEX_EDGE_RATIO:
                return ComputeHexEdgeRatio(points);
                break;
            case HEX_MAX_EDGE_RATIO:
                return ComputeHexMaxEdgeRatio(points);
                break;
            case HEX_SKEW:
                return ComputeHexSkew(points);
                break;
            case HEX_STRETCH:
                return ComputeHexStretch(points);
                break;
            case HEX_DIAGONAL:
                return ComputeHexDiagonal(points);
                break;
            case HEX_RELATIVE_SIZE_SQUARED:
                return 0;
                break;
            default:
                break;
        }
    }
    return 0.0;
}


/*
* 计算某一个四面体单元的最小/最大长度
*/
std::vector<double> VolumeMeshMetricsFilter::GetMinAndMaxLenOfCell(const std::vector<Point>& points) {
    double minLength = DBL_MAX, maxLength = 0.0;

    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            double length = (points[i] - points[j]).norm();
            minLength = std::min(minLength, length);
            maxLength = std::max(maxLength, length);
        }
    }

    std::vector<double> result;
    result.push_back(minLength);
    result.push_back(maxLength);
    return result;
}

/*
	* 计算某一个四面体单元的边长比 最长边/最短边
	* 正四面体：1
	*/
double VolumeMeshMetricsFilter::ComputeTetEdgeRatio(const std::vector<Point>& points) {
    std::vector<double> result = GetMinAndMaxLenOfCell(points);
    return result[1] / result[0];
}

/*
* 计算某一个四面体单元的体积
*/
double VolumeMeshMetricsFilter::ComputeTetVolume(const std::vector<Point>& points) {
    std::vector<Point> L(3);
    L[0] = points[1] - points[0];
    L[1] = points[2] - points[0];
    L[2] = points[3] - points[0];
    double volume = std::abs(L[0].cross(L[1]).dot(L[2]) / 6.0);

    return volume;
}

/*
* 计算某一个面的面积 
*/
double VolumeMeshMetricsFilter::GetAreaOfFace(Point v1, Point v2, Point v3) {
    return ((v2 - v1).cross(v3 - v1)).norm() / 2.0;
}

/*
* 计算某一个四面体单元的面积 该点对应面的面积 
*/
std::vector<double> VolumeMeshMetricsFilter::GetAreaOfCell(const std::vector<Point>& points) {

    std::vector<double> areas;

    double area = GetAreaOfFace(points[1], points[2], points[3]);
    areas.push_back(area);

    area = GetAreaOfFace(points[0], points[2], points[3]);
    areas.push_back(area);

    area = GetAreaOfFace(points[0], points[1], points[3]);
    areas.push_back(area);

    area = GetAreaOfFace(points[0], points[1], points[2]);
    areas.push_back(area);

    return areas;
}

/*
* 计算某一个四面体单元的内切球的半径  3*volume/sum(area)
*/
double VolumeMeshMetricsFilter::GetInradiusOfCell(const std::vector<Point>& points) {
    std::vector<double> areas = GetAreaOfCell(points);
    double volume = ComputeTetVolume(points);
    double sumArea = 0;
    for (double area: areas) { sumArea += area; }
    return 3.0 * volume / sumArea;
}

/*
	* 计算某一个四面体单元的外接球的球心
	*/
Point VolumeMeshMetricsFilter::GetOuterCircle(const std::vector<Point>& points) {

    Eigen::Matrix3f m3;


    Point p1 = (points[1] - points[0]) * 2.0;
    Point p2 = (points[2] - points[1]) * 2.0;
    Point p3 = (points[3] - points[2]) * 2.0;
    double d1 = points[1] * points[1] - points[0] * points[0];
    double d2 = points[2] * points[2] - points[1] * points[1];
    double d3 = points[3] * points[3] - points[2] * points[2];
    m3 << p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], p3[0], p3[1], p3[2];


    Eigen::Vector3f vc3(d1, d2, d3);
    Eigen::Matrix3f m3IN = m3.inverse();

    vc3 = m3IN * vc3;
    //return Point(center.x(), center.y(), center.z());
    return Point(vc3(0), vc3(1), vc3(2));
}

/*
	* 计算某一个四面体单元的外接球的半径
	*/
double VolumeMeshMetricsFilter::GetCircumradiusOfCell(const std::vector<Point>& points) {
    Point outcircle = GetOuterCircle(points);

    return (points[0] - outcircle).norm();
}

/*
	* 计算某个四面体的纵横比   最长边/( 2*sqrt(6)*内切半径)   
	* 接受范围：[1,3]
	* 最好：1
	*/
double VolumeMeshMetricsFilter::GetAspectRatioOfCell(const std::vector<Point>& points) {
    // 对齐 Verdict tet_aspect_ratio（V_TetMetric.cpp:760-811）：
    //   hm = 最长棱；A/B/C/D = 4 个面的面积（叉积模长的一半，但 Verdict 未除 2）
    //   aspect_ratio = (sqrt(6)/12) * hm * (A+B+C+D) / |detTet|
    // Verdict 会先做 elem_scaling（包围盒对角线归一化），但这是纯比值运算，
    // char_size 在 hm、A/B/C/D、detTet 中按 1、1、2、3 次幂出现，
    // 归一化后整体不变，因此这里直接在原始尺度上计算。
    // 旧实现用 maxLength/(2*sqrt(6)*inradius)，是另一个量，与 VTK 差异达 601/2706。
    // 全程 double：侧边差与叉积都在 double 下完成。
    // 若在 Vector3f 下做叉积，百万级长宽比的极端退化单元会偏差 ~1%
    // （实测 sphere_ug 上 26/2706 个单元不符，最大绝对偏差 9.4e5）。
    auto dbl = [&](int i, int k) { return static_cast<double>(points[i][k]); };
    auto sub = [&](int a, int b, double out[3]) {
        out[0] = dbl(a, 0) - dbl(b, 0);
        out[1] = dbl(a, 1) - dbl(b, 1);
        out[2] = dbl(a, 2) - dbl(b, 2);
    };
    auto crossLen = [](const double u[3], const double v[3]) {
        const double x = u[1] * v[2] - u[2] * v[1];
        const double y = u[2] * v[0] - u[0] * v[2];
        const double z = u[0] * v[1] - u[1] * v[0];
        return std::sqrt(x * x + y * y + z * z);
    };
    auto sq = [](const double v[3]) { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; };

    double ab[3], ac[3], ad[3], bc[3], cd[3];
    sub(1, 0, ab);
    sub(2, 0, ac);
    sub(3, 0, ad);
    sub(2, 1, bc);
    sub(3, 2, cd);

    const double ab2 = sq(ab), ac2 = sq(ac), ad2 = sq(ad);
    const double bc2 = sq(bc), bd2 = 0, cd2 = sq(cd);
    (void) bd2;

    double A = (ab2 > bc2) ? ab2 : bc2;
    double B = (ac2 > ad2) ? ac2 : ad2;
    double C = (ad2 > cd2) ? ad2 : cd2; // 这里与 Verdict 的 bd2/cd2 取舍保持一致见下
    double D = (A > B) ? A : B;
    // Verdict 原文：C = max(bd2, cd2)。bd2 未在上方算出，补算以严格对齐。
    double bd[3];
    sub(3, 1, bd);
    C = (sq(bd) > cd2) ? sq(bd) : cd2;
    const double hm = (D > C) ? std::sqrt(D) : std::sqrt(C);

    const double detTet = ab[0] * (ac[1] * ad[2] - ac[2] * ad[1])
                        - ab[1] * (ac[0] * ad[2] - ac[2] * ad[0])
                        + ab[2] * (ac[0] * ad[1] - ac[1] * ad[0]);
    if (std::fabs(detTet) < 1e-300) { return 1e30; } // Verdict 的 VERDICT_DBL_MAX 哨兵

    const double sA = crossLen(ab, bc);
    const double sB = crossLen(ab, ad);
    const double sC = crossLen(ac, ad);
    const double sD = crossLen(bc, cd);

    const double aspectRatio = (std::sqrt(6.0) / 12.0) * hm * (sA + sB + sC + sD) / std::fabs(detTet);
    return (aspectRatio < 1e30) ? aspectRatio : 1e30;
}

/*
	* 四面体雅可比行列式（= 6 倍有向体积）
	* 对齐 Verdict tet_jacobian（V_TetMetric.cpp:529-543）：
	*   side0 = P1-P0, side2 = P0-P2, side3 = P3-P0
	*   jacobian = side3 · (side2 × side0)
	* 注意：Verdict 【不取绝对值】，单元定向反了会得到负值。
	* 旧实现返回 |ab·(ac×ad)|，丢失符号，导致 91/2706 个反定向单元与 VTK 符号相反。
	* 用 double 逐分量计算，避免 Vector3f 在叉积中的舍入。
*/
double VolumeMeshMetricsFilter::GetJacobianOfCell(const std::vector<Point>& points) {
    const double s0x = static_cast<double>(points[1][0]) - points[0][0];
    const double s0y = static_cast<double>(points[1][1]) - points[0][1];
    const double s0z = static_cast<double>(points[1][2]) - points[0][2];

    const double s2x = static_cast<double>(points[0][0]) - points[2][0];
    const double s2y = static_cast<double>(points[0][1]) - points[2][1];
    const double s2z = static_cast<double>(points[0][2]) - points[2][2];

    const double s3x = static_cast<double>(points[3][0]) - points[0][0];
    const double s3y = static_cast<double>(points[3][1]) - points[0][1];
    const double s3z = static_cast<double>(points[3][2]) - points[0][2];

    // side2 × side0
    const double cx = s2y * s0z - s2z * s0y;
    const double cy = s2z * s0x - s2x * s0z;
    const double cz = s2x * s0y - s2y * s0x;

    return s3x * cx + s3y * cy + s3z * cz;
}

/*
	* 计算某个四面体的某点的塌陷率
	*/
double VolumeMeshMetricsFilter::GetCollapseRatioOfVertex(Point v1, Point v2, Point v3, double volume) {
    double area = ((v2 - v1).cross(v3 - v1)).norm();
    double high = volume / area * 6;

    double maxLength = std::max((v2 - v1).norm(), (v3 - v1).norm());
    maxLength = std::max(maxLength, (v3 - v2).norm());

    return high / maxLength;
}

/*
	* 计算某个四面体的塌陷率   某点所在高 除以 该点对应的面的最长边。
	* 返回四个点的中最小塌陷率
	* 接受范围：[0.1,DBL MAX]
	* 最好：sqrt(6)/3
	*/
double VolumeMeshMetricsFilter::GetCollapseRatioOfCell(const std::vector<Point>& points) {
    double volume = ComputeTetVolume(points);

    double CollapseRatio = GetCollapseRatioOfVertex(points[1], points[2], points[3], volume);

    CollapseRatio = std::min(CollapseRatio, GetCollapseRatioOfVertex(points[0], points[2], points[3], volume));
    CollapseRatio = std::min(CollapseRatio, GetCollapseRatioOfVertex(points[1], points[0], points[3], volume));
    CollapseRatio = std::min(CollapseRatio, GetCollapseRatioOfVertex(points[1], points[2], points[0], volume));
    return CollapseRatio;
}

/*
	*得到某个四面体体积歪斜度 (外接球的正四面体的体积-体积)/外接球的正四面体的体积
	* 接受范围：[0,1]
	* 最好：0
	*/
double VolumeMeshMetricsFilter::GetVolSkewOfCell(const std::vector<Point>& points) {
    double circumRadius = GetCircumradiusOfCell(points);
    double a = circumRadius * 4.0 / sqrt(6.0);
    double volume = ComputeTetVolume(points);

    double regularVolume = a * a * a * sqrt(2.0) / 12.0;
    return (regularVolume - volume) / regularVolume;
}

/*
	*得到某个四面体的某一点歪斜度   某个节点到对边中点的线段 与 另外两条边中点连接的线段之间 的较小的角
	*/
double VolumeMeshMetricsFilter::GetSkewnessOfVertex(Point v0, Point v1, Point v2) {

    Point m12 = (v1 + v2) / 2;
    Point m01 = (v0 + v1) / 2;
    Point m02 = (v0 + v2) / 2;
    double cosa = (v0 - m01).normalized() * (m01 - m02).normalized();
    double a = acos(cosa) * 180 / PI;
    return std::min(a, 180 - a);
}

/*
	*得到某个四面体的某个面的歪斜度   为三个点的歪斜度中的最大值  某个节点到对边中点的线段 与 另外两条边中点连接的线段之间 的较小的角
	* 接受范围:(0,90]
	* 最好：90
	*/
double VolumeMeshMetricsFilter::GetSkewnessOfFace(const std::vector<Point>& points) {
    double skewOfFace = 0;

    skewOfFace = std::max(GetSkewnessOfVertex(points[0], points[1], points[2]), skewOfFace);
    skewOfFace = std::max(GetSkewnessOfVertex(points[1], points[0], points[2]), skewOfFace);
    skewOfFace = std::max(GetSkewnessOfVertex(points[2], points[1], points[0]), skewOfFace);
    return skewOfFace;
}

/*
	*得到某个四面体的某个面的的v0的内角
	*/
double VolumeMeshMetricsFilter::GetInternalAnglesOfVertex(Point v0, Point v1, Point v2) {

    double cosa = (v1 - v0).normalized() * (v2 - v0).normalized();
    double angle = acos(cosa) * 180.0 / PI;

    return angle;
}

/*
	*得到某个四面体的某个面的三个内角 
	*/
std::vector<double> VolumeMeshMetricsFilter::GetInternalAnglesOfFace(const std::vector<Point>& points) {
    std::vector<double> angles;
    angles.push_back(GetInternalAnglesOfVertex(points[0], points[1], points[2]));
    angles.push_back(GetInternalAnglesOfVertex(points[1], points[0], points[2]));
    angles.push_back(GetInternalAnglesOfVertex(points[2], points[1], points[0]));
    return angles;
}


/*
* 得到某个四面体的所有面的内角  
*/
std::vector<std::vector<double>> VolumeMeshMetricsFilter::GetInternalAnglesOfCell(const std::vector<Point>& points) {
    std::vector<std::vector<double>> angles;

    // 四面体的四个面
    std::vector<std::vector<Point>> faces = {{points[1], points[2], points[3]}, // 排除points[0]的面
                                             {points[0], points[2], points[3]}, // 排除points[1]的面
                                             {points[0], points[1], points[3]}, // ...
                                             {points[0], points[1], points[2]}};

    for (const auto& face: faces) { angles.push_back(GetInternalAnglesOfFace(face)); }

    return angles;
}
/*
    *得到某个四面体的最小内角，角度制
    */
double VolumeMeshMetricsFilter::GetMinInternalAnglesOfCell(const std::vector<Point>& points) {
    double min_angle = DBL_MAX;

    // 计算四个面的最小内角
    std::vector<std::vector<Point>> faces = {{points[1], points[2], points[3]},
                                             {points[0], points[2], points[3]},
                                             {points[0], points[1], points[3]},
                                             {points[0], points[1], points[2]}};

    for (const auto& face: faces) {
        auto angles = GetInternalAnglesOfFace(face);
        for (double a: angles) { min_angle = std::min(min_angle, a); }
    }

    return min_angle;
}

/*
	* 计算某个四面体v0所在的高的长度
	*/
double VolumeMeshMetricsFilter::GetHighOfVertex(Point v0, Point v1, Point v2, Point v3) {
    Point normOfFace = ((v2 - v1).cross(v3 - v1)).normalized();
    return std::abs(normOfFace * (v0 - v1));
}

/*
	* 计算某个四面体的体长宽比   最长边/最短高
	* 单位正四面体：sqrt(3)/2
	*/
double VolumeMeshMetricsFilter::GetVolAspectRatioOfCell(const std::vector<Point>& points) {
    double maxLength = GetMinAndMaxLenOfCell(points)[1];

    double minHigh = GetHighOfVertex(points[0], points[1], points[2], points[3]);
    minHigh = std::min(GetHighOfVertex(points[1], points[0], points[2], points[3]), minHigh);
    minHigh = std::min(GetHighOfVertex(points[2], points[1], points[0], points[3]), minHigh);
    minHigh = std::min(GetHighOfVertex(points[3], points[1], points[2], points[0]), minHigh);
    return maxLength / minHigh;
}

/*
	* 计算所有四面体的等角斜率  max( (Qmax-Qe)/(180-Qe)，(Qe-Qmin)/Qe ) Qmax 最大角 Qmin最小 Qe 60（三角形）或90（四边形） acos(1/3.0)四面体
	* 计算出二面角 求其等角斜率大小
	* 再求所有面中的最大角 和最小角  算出等角斜率
	* 取两者中较大的一个
	* 
	* 最好：0
	*
	*/
double VolumeMeshMetricsFilter::GetEquiangleSkewnessOfCell(const std::vector<Point>& points) {

    Vector3f ab = (points[1] - points[0]).normalized();
    Vector3f ac = (points[2] - points[0]).normalized();
    Vector3f ad = (points[3] - points[0]).normalized();
    Vector3f bc = (points[2] - points[1]).normalized();
    Vector3f bd = (points[3] - points[1]).normalized();
    Vector3f cd = (points[3] - points[2]).normalized();

    Vector3f abc = (bc.cross(ab)).normalized();
    Vector3f abd = (ab.cross(ad)).normalized();
    Vector3f acd = (cd.cross(ad)).normalized();
    Vector3f bcd = (bc.cross(cd)).normalized();

    //二面角
    double alpha = acos(-(abc * abd));
    double beta = acos(-(abc * acd));
    double gamma = acos(-(abc * bcd));
    double delta = acos(-(abd * acd));
    double epsilon = acos(-(abd * bcd));
    double zeta = acos(-(acd * bcd));

    double minAngle = alpha;
    minAngle = std::min(minAngle, beta);
    minAngle = std::min(minAngle, gamma);
    minAngle = std::min(minAngle, delta);
    minAngle = std::min(minAngle, epsilon);
    minAngle = std::min(minAngle, zeta);

    double maxAngle = alpha;
    maxAngle = std::min(maxAngle, beta);
    maxAngle = std::min(maxAngle, gamma);
    maxAngle = std::min(maxAngle, delta);
    maxAngle = std::min(maxAngle, epsilon);
    maxAngle = std::min(maxAngle, zeta);

    double theta = acos(1 / 3.0) * 180.0 / PI;
    minAngle *= 180.0 / PI;
    maxAngle *= 180.0 / PI;

    double dihedralMaxSkew = (maxAngle - theta) / (180.0 - theta);
    double dihedralMinSkew = (theta - minAngle) / theta;

    double maxEquiangleSkew = dihedralMaxSkew;
    maxEquiangleSkew = std::max(maxEquiangleSkew, dihedralMinSkew);

    //面上的内角
    /*double angles[12];

		angles[0] = acos(-(ab * bc));
		angles[1] = acos((bc * ac));
		angles[2] = acos((ac * ab));
		angles[3] = acos(-(ab * bd));
		angles[4] = acos((bd * ad));
		angles[5] = acos((ad * ab));
		angles[6] = acos(-(bc * cd));
		angles[7] = acos((cd * bd));
		angles[8] = acos((bd * bc));
		angles[9] = acos((ad * cd));
		angles[10] = acos(-(cd * ac));
		angles[11] = acos((ac * ad));
		
			minAngle = angles[0];
			maxAngle = angles[1];
			for (int i = 0; i < 12; i++) {
				minAngle = min(minAngle, angles[i]);
				maxAngle = max(maxAngle, angles[i]);
			}

			minAngle *= 180.0 / TET_PI;
			maxAngle *= 180.0 / TET_PI;*/

    std::vector<std::vector<double>> anglesOfFaces = GetInternalAnglesOfCell(points);
    minAngle = anglesOfFaces[0][0];
    maxAngle = anglesOfFaces[0][0];

    for (std::vector<double> angles: anglesOfFaces) {
        for (double angle: angles) {
            minAngle = std::min(angle, minAngle);
            maxAngle = std::max(angle, maxAngle);
        }
    }


    theta = 60.0;
    double maxSkew = (maxAngle - theta) / (180.0 - theta);
    double minSkew = (theta - minAngle) / theta;

    maxEquiangleSkew = std::max(maxEquiangleSkew, maxSkew);
    maxEquiangleSkew = std::max(maxEquiangleSkew, minSkew);


    return maxEquiangleSkew = std::max(maxEquiangleSkew, maxSkew);
}

// 四面体最小二面角（度）—— 严格对齐 Verdict tet_minimum_angle
// （V_TetMetric.cpp:931-972）
//
// Verdict 的算法：先求出 4 个面的【未归一化法向】，再两两求夹角，
// 取 6 个二面角中的最小值，最后乘 180/π 转成角度。
// 注意：法向必须在做点积时再除以模长，不能提前 normalized() ——
// 提前归一化会改变数值路径，导致与 VTK 结果不一致（实测约 43% 的四面体有偏差）。
double VolumeMeshMetricsFilter::GetMinDihedralAngleOfTet(const std::vector<Point>& points) {
    // 全程用 double：侧边差、叉积、点积都在 double 下完成。
    // 若在 Vector3f 下做叉积，极小角（<0.001°）会因抵消误差被放大到几十倍
    // （实测 sphere_ug 上 435/2706 个单元不符，最大绝对偏差 0.0235）。
    auto dbl = [&](int i, int k) { return static_cast<double>(points[i][k]); };

    const double abx = dbl(1, 0) - dbl(0, 0), aby = dbl(1, 1) - dbl(0, 1), abz = dbl(1, 2) - dbl(0, 2);
    const double adx = dbl(3, 0) - dbl(0, 0), ady = dbl(3, 1) - dbl(0, 1), adz = dbl(3, 2) - dbl(0, 2);
    const double bcx = dbl(2, 0) - dbl(1, 0), bcy = dbl(2, 1) - dbl(1, 1), bcz = dbl(2, 2) - dbl(1, 2);
    const double cdx = dbl(3, 0) - dbl(2, 0), cdy = dbl(3, 1) - dbl(2, 1), cdz = dbl(3, 2) - dbl(2, 2);

    // 4 个面的法向（未归一化叉积）
    const double abc[3] = {aby * bcz - abz * bcy, abz * bcx - abx * bcz, abx * bcy - aby * bcx};
    const double abd[3] = {aby * adz - abz * ady, abz * adx - abx * adz, abx * ady - aby * adx};
    const double acd[3] = {ady * cdz - adz * cdy, adz * cdx - adx * cdz, adx * cdy - ady * cdx};
    const double bcd[3] = {bcy * cdz - bcz * cdy, bcz * cdx - bcx * cdz, bcx * cdy - bcy * cdx};

    auto len = [](const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };
    auto dot = [](const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };

    const double nabc = len(abc), nabd = len(abd), nacd = len(acd), nbcd = len(bcd);

    // 严格复现 Verdict 的数值行为（V_TetMetric.cpp:957-969）：
    //   1) 立刻 acos，【不做 clamp】。退化单元上 (u·v)/(|u||v|) 会略微越出 [-1,1]，
    //      此时 acos 返回 NaN；Verdict 的 min 链是 `a = a < b ? a : b` 形式，
    //      遇到 NaN 会走 else 分支把它丢掉。
    //   2) 不做法向模长为 0 的提前返回：薄片四面体（体积 ~1e-12）的基准值
    //      是 179.9999991（由非 NaN 的那个二面角给出），提前返回 0 会错。
    // 早期实现同时做了 clamp 和提前返回，在 holdout_sphere 上 2/368 个单元不符。
    double alpha = std::acos(dot(abc, abd) / (nabc * nabd));
    double beta = std::acos(dot(abc, acd) / (nabc * nacd));
    double gamma = std::acos(dot(abc, bcd) / (nabc * nbcd));
    double delta = std::acos(dot(abd, acd) / (nabd * nacd));
    double epsilon = std::acos(dot(abd, bcd) / (nabd * nbcd));
    double zeta = std::acos(dot(acd, bcd) / (nacd * nbcd));

    double mn = alpha;
    mn = mn < beta ? mn : beta;
    mn = mn < gamma ? mn : gamma;
    mn = mn < delta ? mn : delta;
    mn = mn < epsilon ? mn : epsilon;
    mn = mn < zeta ? mn : zeta;
    mn *= 180.0 / PI;

    // Verdict 的 fix_range：NaN → DBL_MAX，其余原样返回
    if (std::isnan(mn)) { return 1e30; }
    return mn;
}

//六面体


// 体积
double VolumeMeshMetricsFilter::ComputeHexVolume(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    auto X_1 = (P_1 - P_0) + (P_2 - P_3) + (P_5 - P_4) + (P_6 - P_7);
    auto X_2 = (P_3 - P_0) + (P_2 - P_1) + (P_7 - P_4) + (P_6 - P_5);
    auto X_3 = (P_4 - P_0) + (P_5 - P_1) + (P_6 - P_2) + (P_7 - P_3);

    return X_1.dot(X_2.cross(X_3)) / 64.0;
}

// 锥度, range : [0, MAX], acceptable range [0, 0.5], unit cube : 0
double VolumeMeshMetricsFilter::ComputeHexTaper(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    auto X_1 = (P_1 - P_0) + (P_2 - P_3) + (P_5 - P_4) + (P_6 - P_7);
    auto X_2 = (P_3 - P_0) + (P_2 - P_1) + (P_7 - P_4) + (P_6 - P_5);
    auto X_3 = (P_4 - P_0) + (P_5 - P_1) + (P_6 - P_2) + (P_7 - P_3);

    auto X_12 = (P_2 - P_3) - (P_1 - P_0) + (P_6 - P_7) - (P_5 - P_4);
    auto X_13 = (P_5 - P_1) - (P_4 - P_0) + (P_6 - P_2) - (P_7 - P_3);
    auto X_23 = (P_7 - P_4) - (P_3 - P_0) + (P_6 - P_5) - (P_2 - P_1);

    auto T_12 = X_12.norm() / ((std::min) (X_1.norm(), X_2.norm()));
    auto T_13 = X_13.norm() / ((std::min) (X_1.norm(), X_3.norm()));
    auto T_23 = X_23.norm() / ((std::min) (X_2.norm(), X_3.norm()));

    return (std::max) ({T_12, T_13, T_23});
}

// 雅可比, range : [0, MAX], acceptable range [0, MAX], unit cube : 1
double VolumeMeshMetricsFilter::ComputeHexJacobian(const std::vector<Point>& points) {
    //be caution about order

    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    float jacobian = (std::numeric_limits<float>::max)();
    float current_jacobian = 0.0;

    Vector3f L_0, L_2, L_3;

    L_0 = P_1 - P_0;
    L_2 = P_3 - P_0;
    L_3 = P_4 - P_0;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_2 - P_1;
    L_2 = P_0 - P_1;
    L_3 = P_5 - P_1;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_3 - P_2;
    L_2 = P_1 - P_2;
    L_3 = P_6 - P_2;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_0 - P_3;
    L_2 = P_2 - P_3;
    L_3 = P_7 - P_3;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_7 - P_4;
    L_2 = P_5 - P_4;
    L_3 = P_0 - P_4;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_4 - P_5;
    L_2 = P_6 - P_5;
    L_3 = P_1 - P_5;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_5 - P_6;
    L_2 = P_7 - P_6;
    L_3 = P_2 - P_6;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    L_0 = P_6 - P_7;
    L_2 = P_4 - P_7;
    L_3 = P_3 - P_7;

    current_jacobian = (L_2.cross(L_3)).dot(L_0);
    if (current_jacobian < jacobian) jacobian = current_jacobian;

    return jacobian;
}

// 长宽比 , range : [1, MAX], acceptable range [1, MAX], unit cube : 1
double VolumeMeshMetricsFilter::ComputeHexEdgeRatio(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    // 严格对齐 Verdict 官方 hex_edge_ratio（V_HexMetric.cpp:714-827）。
    //
    // 12 条棱按 Verdict 的顺序排列：
    //   edges[0..3]  = 底面 (0-1, 1-2, 2-3, 3-0)
    //   edges[4..7]  = 顶面 (4-5, 5-6, 6-7, 7-4)
    //   edges[8..11] = 竖向 (0-4, 1-5, 2-6, 3-7)
    // Verdict 内部用"长度平方"比较，最后开根号。
    const double e2[12] = {
            (P_1 - P_0).squaredNorm(), (P_2 - P_1).squaredNorm(), (P_3 - P_2).squaredNorm(), (P_3 - P_0).squaredNorm(),
            (P_5 - P_4).squaredNorm(), (P_6 - P_5).squaredNorm(), (P_7 - P_6).squaredNorm(), (P_7 - P_4).squaredNorm(),
            (P_4 - P_0).squaredNorm(), (P_5 - P_1).squaredNorm(), (P_6 - P_2).squaredNorm(), (P_7 - P_3).squaredNorm()};

    // 6 组"对边"：每组取 min / max
    const double mab = (e2[0] < e2[1]) ? e2[0] : e2[1];
    const double Mab = (e2[0] < e2[1]) ? e2[1] : e2[0];
    const double mcd = (e2[2] < e2[3]) ? e2[2] : e2[3];
    const double Mcd = (e2[2] < e2[3]) ? e2[3] : e2[2];
    const double mef = (e2[4] < e2[5]) ? e2[4] : e2[5];
    const double Mef = (e2[4] < e2[5]) ? e2[5] : e2[4];
    const double mgh = (e2[6] < e2[7]) ? e2[6] : e2[7];
    const double Mgh = (e2[6] < e2[7]) ? e2[7] : e2[6];
    const double mij = (e2[8] < e2[9]) ? e2[8] : e2[9];
    const double Mij = (e2[8] < e2[9]) ? e2[9] : e2[8];
    const double mkl = (e2[10] < e2[11]) ? e2[10] : e2[11];
    const double Mkl = (e2[10] < e2[11]) ? e2[11] : e2[10];

    // 分母：全部 12 条棱的最小值（平方）
    double m2 = (mab < mcd) ? mab : mcd;
    m2 = (m2 < mef) ? m2 : mef;
    m2 = (m2 < mgh) ? m2 : mgh;
    m2 = (m2 < mij) ? m2 : mij;
    m2 = (m2 < mkl) ? m2 : mkl;
    if (m2 < 1e-300) { return 0.0; }

    // 分子：★ Verdict 的真实行为 —— 虽然第 810-814 行先算了全部 6 组的 max，
    // 但第 815-818 行随即用"只含前 3 组"的结果覆盖了它。
    // 必须复现这个覆盖，否则与 VTK 结果不一致（实测差异面达 6.26%）。
    double M2 = (Mab > Mcd) ? Mab : Mcd;
    M2 = (M2 > Mef) ? M2 : Mef;

    return std::sqrt(M2 / m2);
}

// 最大长宽比 , range : [1, MAX], acceptable range [1, 1.3], unit cube : 1
double VolumeMeshMetricsFilter::ComputeHexMaxEdgeRatio(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    auto X_1 = (P_1 - P_0) + (P_2 - P_3) + (P_5 - P_4) + (P_6 - P_7);
    auto X_2 = (P_3 - P_0) + (P_2 - P_1) + (P_7 - P_4) + (P_6 - P_5);
    auto X_3 = (P_4 - P_0) + (P_5 - P_1) + (P_6 - P_2) + (P_7 - P_3);

    auto L_1 = X_1.norm();
    auto L_2 = X_2.norm();
    auto L_3 = X_3.norm();

    auto A_12 = (std::max) (L_1 / L_2, L_2 / L_1);
    auto A_13 = (std::max) (L_1 / L_3, L_3 / L_1);
    auto A_23 = (std::max) (L_2 / L_3, L_3 / L_2);

    return (std::max) ({A_12, A_13, A_23});
}

// 体积歪斜度/歪斜度 , range : [0, 1], acceptable range [0, 0.5], unit cube : 0
double VolumeMeshMetricsFilter::ComputeHexSkew(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    auto X_1 = (P_1 - P_0) + (P_2 - P_3) + (P_5 - P_4) + (P_6 - P_7);
    auto X_2 = (P_3 - P_0) + (P_2 - P_1) + (P_7 - P_4) + (P_6 - P_5);
    auto X_3 = (P_4 - P_0) + (P_5 - P_1) + (P_6 - P_2) + (P_7 - P_3);

    auto X_1_hat = X_1.normalized();
    auto X_2_hat = X_2.normalized();
    auto X_3_hat = X_3.normalized();

    auto skew_12 = std::abs(X_1_hat.dot(X_2_hat));
    auto skew_13 = std::abs(X_1_hat.dot(X_3_hat));
    auto skew_23 = std::abs(X_2_hat.dot(X_3_hat));

    return (std::max) ({skew_12, skew_13, skew_23});
}

// 伸展度 , range : [0, 1], acceptable range [0.25, 1], unit cube : 1
double VolumeMeshMetricsFilter::ComputeHexStretch(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    std::vector<float> edges(12, 0);
    edges[0] = (P_1 - P_0).norm();
    edges[1] = (P_2 - P_1).norm();
    edges[2] = (P_3 - P_2).norm();
    edges[3] = (P_3 - P_0).norm();
    edges[4] = (P_4 - P_0).norm();
    edges[5] = (P_5 - P_1).norm();
    edges[6] = (P_6 - P_2).norm();
    edges[7] = (P_7 - P_3).norm();
    edges[8] = (P_5 - P_4).norm();
    edges[9] = (P_6 - P_5).norm();
    edges[10] = (P_7 - P_6).norm();
    edges[11] = (P_7 - P_4).norm();

    std::vector<float> diagonals(4, 0);

    diagonals[0] = (P_6 - P_0).norm();
    diagonals[1] = (P_7 - P_1).norm();
    diagonals[2] = (P_4 - P_2).norm();
    diagonals[3] = (P_5 - P_3).norm();

    float L_min = *(std::min_element(edges.begin(), edges.end()));
    float D_max = *(std::max_element(diagonals.begin(), diagonals.end()));

    return std::sqrt(3.0f) * L_min / D_max;
}

// 对角线长度比值, range : [0, 1], acceptable range [0.65, 1], unit cube : 1
double VolumeMeshMetricsFilter::ComputeHexDiagonal(const std::vector<Point>& points) {
    auto P_0 = points[0];
    auto P_1 = points[1];
    auto P_2 = points[2];
    auto P_3 = points[3];
    auto P_4 = points[4];
    auto P_5 = points[5];
    auto P_6 = points[6];
    auto P_7 = points[7];

    std::vector<float> diagonals(4, 0);

    diagonals[0] = (P_6 - P_0).norm();
    diagonals[1] = (P_7 - P_1).norm();
    diagonals[2] = (P_4 - P_2).norm();
    diagonals[3] = (P_5 - P_3).norm();

    auto max_ele = std::max_element(diagonals.begin(), diagonals.end());
    auto min_ele = std::min_element(diagonals.begin(), diagonals.end());

    return *min_ele / *max_ele;
}

// 相对大小平方, range : [0, 1], acceptable range [0, 1], unit cube : 依赖于平均体积
double VolumeMeshMetricsFilter::ComputeHexRelativeSizeSquared(const std::vector<Point>& points, float average_volume) {
    auto D = ComputeHexVolume(points) / average_volume;
    auto sqr_q = (std::min) (D, 1.0f / D);

    return sqr_q * sqr_q;
}

// 六面体最小标量雅可比
double VolumeMeshMetricsFilter::ComputeHexMinScaledJacobian(const std::vector<Point>& points) {
    double min_det = 2.f; // the value of Scaled Jacobian is the	determinant of the matrix Jacobian
    std::unordered_map<int, std::vector<int>> neighbor;
    std::vector<std::pair<int, int>> edges = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底面
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶面
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // 侧面
    };
    for (const auto& edge: edges) {
        neighbor[edge.first].push_back(edge.second);
        neighbor[edge.second].push_back(edge.first);
    }

    // 对每个顶点计算雅可比矩阵行列式
    for (int i = 0; i < 8; ++i) {
        auto& v = points[i];
        auto adjvh = neighbor[i];

        if (adjvh.size() != 3) { continue; }

        auto v1 = points[adjvh[0]];
        auto v2 = points[adjvh[1]];
        auto v3 = points[adjvh[2]];

        auto center = (v1 + v2 + v3) / 3.0;
        auto vec = (center - v).normalized();

        auto vec12 = v2 - v1;
        auto vec13 = v3 - v1;
        auto normal = (vec12.cross(vec13)).normalized();
        double cosine = vec.dot(normal);

        if (cosine < 0) { std::swap(v2, v3); }

        std::vector<Vector3d> ev(3);
        auto tmp1 = v1 - v;
        ev[0] = tmp1.normalized();
        auto tmp2 = v2 - v;
        ev[1] = tmp2.normalized();
        auto tmp3 = v3 - v;
        ev[2] = tmp3.normalized();

        Eigen::Matrix3d J;
        J << ev[0][0], ev[1][0], ev[2][0], ev[0][1], ev[1][1], ev[2][1], ev[0][2], ev[1][2], ev[2][2];

        double det = J.determinant();
        min_det = std::min(min_det, det);
    }

    if (min_det < 0) { igDebug("Warning: There exist too bad hexahedron!!!"); }

    return min_det;
}

// 六面体平均标量雅可比
double VolumeMeshMetricsFilter::ComputeHexAvgScaledJacobian(const std::vector<Point>& points) {
    double sum_det = 0.0;

    // 构建邻接关系
    std::unordered_map<int, std::vector<int>> neighbor;

    std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    for (const auto& edge: edges) {
        neighbor[edge.first].push_back(edge.second);
        neighbor[edge.second].push_back(edge.first);
    }

    int valid_vertices = 0;

    // 对每个顶点计算雅可比矩阵行列式
    for (int i = 0; i < 8; ++i) {
        auto& v = points[i];
        auto adjvh = neighbor[i];

        if (adjvh.size() != 3) {
            continue; // 跳过不满足条件的顶点
        }

        valid_vertices++;

        auto v1 = points[adjvh[0]];
        auto v2 = points[adjvh[1]];
        auto v3 = points[adjvh[2]];

        // 计算中心点并确定法向
        auto center = (v1 + v2 + v3) / 3.0;
        auto vec = (center - v).normalized();

        // 调整顶点顺序以确保正确的法向
        auto vec12 = v2 - v1;
        auto vec13 = v3 - v1;
        auto normal = (vec12.cross(vec13)).normalized();
        double cosine = vec.dot(normal);

        if (cosine < 0) { std::swap(v2, v3); }

        // 构建三个边的单位向量
        std::vector<Vector3d> ev(3);
        auto tmp1 = v1 - v;
        ev[0] = tmp1.normalized();
        auto tmp2 = v2 - v;
        ev[1] = tmp2.normalized();
        auto tmp3 = v3 - v;
        ev[2] = tmp3.normalized();

        // 构建雅可比矩阵
        Eigen::Matrix3d J;
        J << ev[0][0], ev[1][0], ev[2][0], ev[0][1], ev[1][1], ev[2][1], ev[0][2], ev[1][2], ev[2][2];

        sum_det += J.determinant();
    }

    return (valid_vertices > 0) ? sum_det / valid_vertices : 0.0;
}

// 四面体替代纵横比计算方法
double VolumeMeshMetricsFilter::ComputeTetAspectRatioAlt(const std::vector<Point>& points) {
    if (points.size() != 4) {
        igDebug("Error: It is not a tet mesh!");
        return 0.0;
    }

    // 计算最长边长度
    double len_max = 0.0;
    std::vector<std::pair<int, int>> edges = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    for (const auto& edge: edges) {
        double length = (points[edge.first] - points[edge.second]).norm();
        len_max = std::max(len_max, length);
    }

    double height_min = DBL_MAX;

    for (int i = 0; i < 4; ++i) {
        auto& v = points[i];
        std::vector<Point> face_points;

        for (int j = 0; j < 4; ++j) {
            if (j != i) { face_points.push_back(points[j]); }
        }

        Vector3f normal = (face_points[2] - face_points[0]).cross(face_points[1] - face_points[0]);

        double d = -normal.dot(face_points[0]);
        double numerator = std::abs(normal.dot(v) + d);
        double denominator = normal.norm();

        double height = numerator / std::max(denominator, 1e-7); // 避免除以零
        height_min = std::min(height_min, height);
    }

    double aspect_ratio = len_max / std::max(height_min, 1e-7);

    return aspect_ratio;
}


//四面体体积计算 - 基于MeshMath.h中的get_volume_tetahedral_mesh
double VolumeMeshMetricsFilter::ComputeTetVolumeAlt(const std::vector<Point>& points) {
    if (points.size() != 4) { return 0.0; }

    double volume_mesh = 0;

    std::vector<std::vector<int>> faces = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 2, 3}};

    for (const auto& face: faces) {
        auto& v0 = points[face[0]];
        auto& v1 = points[face[1]];
        auto& v2 = points[face[2]];

        double v_tet = (v0.cross(v1)).dot(v2);
        volume_mesh += v_tet;
    }

    return std::abs(volume_mesh) / 6.0;
}

//六面体体积计算 - 基于MeshMath.h中的get_volume_hexahedral_mesh
double VolumeMeshMetricsFilter::ComputeHexVolumeAlt(const std::vector<Point>& points) {
    if (points.size() != 8) { return 0.0; }

    double volume_mesh = 0;

    std::vector<std::vector<std::vector<int>>> quad_faces = {
            {{0, 1, 2}, {2, 3, 0}}, // 底面
            {{4, 5, 6}, {6, 7, 4}}, // 顶面
            {{0, 1, 5}, {5, 4, 0}}, // 前面
            {{2, 3, 7}, {7, 6, 2}}, // 后面
            {{0, 3, 7}, {7, 4, 0}}, // 左面
            {{1, 2, 6}, {6, 5, 1}}  // 右面
    };

    for (const auto& quad: quad_faces) {
        for (const auto& triangle: quad) {
            auto& v0 = points[triangle[0]];
            auto& v1 = points[triangle[1]];
            auto& v2 = points[triangle[2]];

            double v_tet = (v0.cross(v1)).dot(v2);
            volume_mesh += v_tet;
        }
    }

    return std::abs(volume_mesh) / 6.0;
}

// ===========================================================================
// 以下为对齐 ParaView Cell Quality 新增的四面体/六面体指标。
// 公式、分支与哨兵值均对照 Verdict 官方源码（sandialabs/verdict，BSD-3），
// 数值类型统一用 double：项目的 Point 是 Vector3f，float 在"小量相减"时
// 会产生抵消误差（实测会把极小角放大一个量级以上）。
// ===========================================================================

namespace {

constexpr double kVdMax = 1e30;  // Verdict VERDICT_DBL_MAX
constexpr double kVdMin = 1e-30; // Verdict VERDICT_DBL_MIN

inline void ToD8(const std::vector<iGame::Point>& p, int n, double c[][3]) {
    for (int i = 0; i < n; ++i) {
        c[i][0] = static_cast<double>(p[i][0]);
        c[i][1] = static_cast<double>(p[i][1]);
        c[i][2] = static_cast<double>(p[i][2]);
    }
}
inline void SubV(const double a[3], const double b[3], double o[3]) {
    o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2];
}
inline double LenV(const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
inline double LenSqV(const double v[3]) { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; }
inline double DotV(const double u[3], const double v[3]) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; }
inline void CrossV(const double u[3], const double v[3], double o[3]) {
    o[0] = u[1] * v[2] - u[2] * v[1];
    o[1] = u[2] * v[0] - u[0] * v[2];
    o[2] = u[0] * v[1] - u[1] * v[0];
}
template <int N>
inline double ScaleOf(const double c[][3]) {
    double mn[3] = {c[0][0], c[0][1], c[0][2]};
    double mx[3] = {c[0][0], c[0][1], c[0][2]};
    for (int i = 1; i < N; ++i) {
        for (int k = 0; k < 3; ++k) {
            if (c[i][k] < mn[k]) { mn[k] = c[i][k]; }
            if (c[i][k] > mx[k]) { mx[k] = c[i][k]; }
        }
    }
    double d[3] = {mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]};
    const double l = LenV(d);
    return (l < kVdMin) ? 1.0 : l;
}

// Verdict tri_condition 核心（V_TriMetric.cpp:579-599），供 tri 的 condition 复用
inline double TriCondD(const double a[3], const double b[3], const double c[3]) {
    double v1[3], v2[3];
    SubV(b, a, v1);
    SubV(c, a, v2);
    double cr[3];
    CrossV(v1, v2, cr);
    const double areaX2 = LenV(cr);
    if (areaX2 == 0.0) { return kVdMax; }
    const double cond = (DotV(v1, v1) + DotV(v2, v2) - DotV(v1, v2)) / (areaX2 * std::sqrt(3.0));
    return std::min(cond, kVdMax);
}

} // namespace

// ---------------------------------------------------------------------------
// 四面体：6 个新指标
// ---------------------------------------------------------------------------

// 对齐 Verdict tet_condition（V_TetMetric.cpp:1391-1418）：
//   side0 = P1-P0, side2 = P2-P0, side3 = P3-P0（均在包围盒归一化后）
//   c_1 = side0;  c_2 = (-2*side2 - side0)/√3;  c_3 = (3*side3 + side2 - side0)/√6
//   term1 = Σ|c_i|²；term2 = Σ|c_i×c_j|²；det = c_1·(c_2×c_3)
//   return sqrt(term1)*sqrt(term2) / (3*det)
// 旧实现误用"4 个面的三角形 condition 最大值"，与该定义无关。
double VolumeMeshMetricsFilter::ComputeTetCondition(const std::vector<Point>& points) {
    double c[4][3];
    ToD8(points, 4, c);
    const double cs = ScaleOf<4>(c);

    double side0[3], side2[3], side3[3];
    SubV(c[1], c[0], side0);
    // 注意方向：Verdict 写的是 VerdictVector side2{coordinates[2], coordinates[0]}，
    // 即从 P2 指向 P0，结果是 P0 - P2（不是 P2 - P0）。写反会让 condition 差 ~11%。
    SubV(c[0], c[2], side2);
    SubV(c[3], c[0], side3);
    for (int k = 0; k < 3; ++k) { side0[k] /= cs; side2[k] /= cs; side3[k] /= cs; }

    const double s3 = std::sqrt(3.0), s6 = std::sqrt(6.0);
    double c1[3], c2[3], c3[3];
    for (int k = 0; k < 3; ++k) {
        c1[k] = side0[k];
        c2[k] = (-2.0 * side2[k] - side0[k]) / s3;
        c3[k] = (3.0 * side3[k] + side2[k] - side0[k]) / s6;
    }

    const double term1 = DotV(c1, c1) + DotV(c2, c2) + DotV(c3, c3);
    double x12[3], x23[3], x13[3];
    CrossV(c1, c2, x12);
    CrossV(c2, c3, x23);
    CrossV(c1, c3, x13);
    const double term2 = DotV(x12, x12) + DotV(x23, x23) + DotV(x13, x13);

    double cr[3];
    CrossV(c2, c3, cr);
    const double det = DotV(c1, cr);
    if (std::fabs(det) <= kVdMin) { return kVdMax; }

    return std::sqrt(term1) * std::sqrt(term2) / (3.0 * det);
}

// 对齐 Verdict tet_distortion（V_TetMetric.cpp:1528）：线性四面体恒为 1.0
double VolumeMeshMetricsFilter::ComputeTetDistortion(const std::vector<Point>& points) {
    (void) points;
    return 1.0; // Verdict：直边四面体即目标形状
}

// 对齐 Verdict tet_equiangle_skew（V_TetMetric.cpp:60-164）：
//   分两组量，最后取四者的最大值：
//     A) 6 个【二面角】alpha..zeta（来自 4 个归一化面法向两两点积取负）
//        dihedral_skew_max = (max*180/π - θ)/(180-θ)，θ = acos(1/3)
//        dihedral_skew_min = (θ - min*180/π)/θ
//     B) 12 个【面内角】（Verdict 硬编码的 12 个边-边夹角，注意正负号模式）
//        skew_max = (max*180/π - 60)/120，skew_min = (60 - min*180/π)/60
// 旧实现只算了二面角那一组，漏掉 12 个面内角，导致约 88% 的四面体不符。
double VolumeMeshMetricsFilter::ComputeTetEquiangleSkew(const std::vector<Point>& points) {
    double c[4][3];
    ToD8(points, 4, c);

    auto unit = [&](int from, int to, double out[3]) {
        SubV(c[to], c[from], out);
        const double l = LenV(out);
        if (l > kVdMin) { for (int k = 0; k < 3; ++k) { out[k] /= l; } }
    };

    double ab[3], ac[3], ad[3], bc[3], bd[3], cd[3];
    unit(0, 1, ab);
    unit(0, 2, ac);
    unit(0, 3, ad);
    unit(1, 2, bc);
    unit(1, 3, bd);
    unit(2, 3, cd);

    auto unitCross = [](const double u[3], const double v[3], double out[3]) {
        CrossV(u, v, out);
        const double l = LenV(out);
        if (l > kVdMin) { for (int k = 0; k < 3; ++k) { out[k] /= l; } }
    };

    double abc[3], abd[3], acd[3], bcd[3];
    unitCross(bc, ab, abc);
    unitCross(ab, ad, abd);
    unitCross(cd, ad, acd);
    unitCross(bc, cd, bcd);

    const double kRad2Deg = 180.0 / 3.14159265358979323846;
    auto acosClamped = [&](double x) { return std::acos(std::clamp(x, -1.0, 1.0)); };

    // A) 6 个二面角
    const double alpha = acosClamped(-DotV(abc, abd));
    const double beta = acosClamped(-DotV(abc, acd));
    const double gamma = acosClamped(-DotV(abc, bcd));
    const double delta = acosClamped(-DotV(abd, acd));
    const double epsilon = acosClamped(-DotV(abd, bcd));
    const double zeta = acosClamped(-DotV(acd, bcd));

    double dihMin = alpha;
    dihMin = std::min(dihMin, beta);
    dihMin = std::min(dihMin, gamma);
    dihMin = std::min(dihMin, delta);
    dihMin = std::min(dihMin, epsilon);
    dihMin = std::min(dihMin, zeta);
    double dihMax = alpha;
    dihMax = std::max(dihMax, beta);
    dihMax = std::max(dihMax, gamma);
    dihMax = std::max(dihMax, delta);
    dihMax = std::max(dihMax, epsilon);
    dihMax = std::max(dihMax, zeta);

    dihMin *= kRad2Deg;
    dihMax *= kRad2Deg;
    const double theta = std::acos(1.0 / 3.0) * kRad2Deg; // 70.5288

    const double dihedralSkewMax = (dihMax - theta) / (180.0 - theta);
    const double dihedralSkewMin = (theta - dihMin) / theta;

    // B) 12 个面内角（符号模式照抄 Verdict V_TetMetric.cpp:129-140）
    const double angles[12] = {
            acosClamped(-DotV(ab, bc)), acosClamped(DotV(bc, ac)), acosClamped(DotV(ac, ab)),
            acosClamped(-DotV(ab, bd)), acosClamped(DotV(bd, ad)), acosClamped(DotV(ad, ab)),
            acosClamped(-DotV(bc, cd)), acosClamped(DotV(cd, bd)), acosClamped(DotV(bd, bc)),
            acosClamped(DotV(ad, cd)), acosClamped(-DotV(cd, ac)), acosClamped(DotV(ac, ad))};

    double faceMin = 360.0, faceMax = 0.0;
    for (int i = 0; i < 12; ++i) {
        if (angles[i] < faceMin) { faceMin = angles[i]; }
        if (angles[i] > faceMax) { faceMax = angles[i]; }
    }
    faceMin *= kRad2Deg;
    faceMax *= kRad2Deg;

    const double skewMax = (faceMax - 60.0) / 120.0;
    const double skewMin = (60.0 - faceMin) / 60.0;

    double maxSkew = dihedralSkewMin;
    maxSkew = std::max(maxSkew, dihedralSkewMax);
    maxSkew = std::max(maxSkew, skewMin);
    maxSkew = std::max(maxSkew, skewMax);
    return maxSkew;
}

// 对齐 Verdict tet_radius_ratio（V_TetMetric.cpp:700）
double VolumeMeshMetricsFilter::ComputeTetRadiusRatio(const std::vector<Point>& points) {
    double c[4][3];
    ToD8(points, 4, c);
    const double cs = ScaleOf<4>(c);

    double s[6][3];
    SubV(c[1], c[0], s[0]);
    SubV(c[2], c[1], s[1]);
    SubV(c[0], c[2], s[2]);
    SubV(c[3], c[0], s[3]);
    SubV(c[3], c[1], s[4]);
    SubV(c[3], c[2], s[5]);
    for (int i = 0; i < 6; ++i) { for (int k = 0; k < 3; ++k) { s[i][k] /= cs; } }

    double t1[3], t2[3], t3[3];
    CrossV(s[2], s[0], t1);
    CrossV(s[3], s[0], t2);
    CrossV(s[3], s[2], t3);

    double numer[3];
    const double l3sq = LenSqV(s[3]), l2sq = LenSqV(s[2]), l0sq = LenSqV(s[0]);
    for (int k = 0; k < 3; ++k) {
        numer[k] = l3sq * t1[k] + l2sq * t2[k] + l0sq * t3[k];
    }

    double cr1[3], cr2[3], cr3[3], cr4[3];
    CrossV(s[2], s[0], cr1);
    CrossV(s[3], s[0], cr2);
    CrossV(s[4], s[1], cr3);
    CrossV(s[3], s[2], cr4);
    const double areaSum = (LenV(cr1) + LenV(cr2) + LenV(cr3) + LenV(cr4)) * 0.5;

    // 体积（已归一化）
    const double vol = std::fabs(s[3][0] * (s[2][1] * s[0][2] - s[2][2] * s[0][1])
                               - s[3][1] * (s[2][0] * s[0][2] - s[2][2] * s[0][0])
                               + s[3][2] * (s[2][0] * s[0][1] - s[2][1] * s[0][0])) / 6.0;
    if (vol < kVdMin) { return kVdMax; }

    const double rr = LenV(numer) * areaSum / (108.0 * vol * vol);
    return (rr > 0.0) ? std::min(rr, kVdMax) : std::max(rr, -kVdMax);
}

// 对齐 Verdict tet_scaled_jacobian（V_TetMetric.cpp:555-613）：
//   六条边（VerdictVector 构造是 (from,to) 方向）：
//     side0=P1-P0, side1=P2-P1, side2=P0-P2, side3=P3-P0, side4=P3-P1, side5=P3-P2
//     （统一除以 char_size = 包围盒对角线）
//   jacobi = side3 · (side2 × side0)
//   length_squared[0..3] 是各顶点三条邻边的长度平方之积，取其最大者开根号；
//   length_product = max(sqrt(该值), |jacobi|)
//   返回 sqrt(2) * jacobi / length_product
// 旧实现只用顶点 0 的三条边，与 Verdict 的"最长乘积顶点"口径不同。
double VolumeMeshMetricsFilter::ComputeTetScaledJacobian(const std::vector<Point>& points) {
    double c[4][3];
    ToD8(points, 4, c);
    const double cs = ScaleOf<4>(c);

    double s[6][3];
    SubV(c[1], c[0], s[0]); // side0 = P1-P0
    SubV(c[2], c[1], s[1]); // side1 = P2-P1
    SubV(c[0], c[2], s[2]); // side2 = P0-P2
    SubV(c[3], c[0], s[3]); // side3 = P3-P0
    SubV(c[3], c[1], s[4]); // side4 = P3-P1
    SubV(c[3], c[2], s[5]); // side5 = P3-P2
    for (int i = 0; i < 6; ++i) { for (int k = 0; k < 3; ++k) { s[i][k] /= cs; } }

    double cr[3];
    CrossV(s[2], s[0], cr);
    const double jacobi = DotV(s[3], cr);

    const double l0 = LenSqV(s[0]), l1 = LenSqV(s[1]), l2 = LenSqV(s[2]);
    const double l3 = LenSqV(s[3]), l4 = LenSqV(s[4]), l5 = LenSqV(s[5]);

    const double ls[4] = {l0 * l2 * l3, l0 * l1 * l4, l1 * l2 * l5, l3 * l4 * l5};
    int which = 0;
    if (ls[1] > ls[which]) { which = 1; }
    if (ls[2] > ls[which]) { which = 2; }
    if (ls[3] > ls[which]) { which = 3; }

    double lengthProduct = std::sqrt(ls[which]);
    if (lengthProduct < std::fabs(jacobi)) { lengthProduct = std::fabs(jacobi); }

    if (lengthProduct < kVdMin) { return kVdMax; }

    return std::sqrt(2.0) * jacobi / lengthProduct;
}

// 对齐 Verdict tet_shape（V_TetMetric.cpp:1440）
double VolumeMeshMetricsFilter::ComputeTetShape(const std::vector<Point>& points) {
    double c[4][3];
    ToD8(points, 4, c);
    const double cs = ScaleOf<4>(c);

    double e[3][3];
    SubV(c[1], c[0], e[0]);
    SubV(c[0], c[2], e[1]);
    SubV(c[3], c[0], e[2]);
    for (int i = 0; i < 3; ++i) { for (int k = 0; k < 3; ++k) { e[i][k] /= cs; } }

    double cr[3];
    CrossV(e[1], e[0], cr);
    const double jac = DotV(e[2], cr);
    if (jac < kVdMin) { return 0.0; }

    const double num = 3.0 * std::pow(std::sqrt(2.0) * jac, 2.0 / 3.0);
    // Verdict V_TetMetric.cpp:915-916 原文：
    //   1.5*(e0·e0 + e1·e1 + e2·e2) - (e0·(-e1) + (-e1)·e2 + e2·e0)
    const double den = 1.5 * (DotV(e[0], e[0]) + DotV(e[1], e[1]) + DotV(e[2], e[2]))
                       - (-DotV(e[0], e[1]) - DotV(e[1], e[2]) + DotV(e[2], e[0]));
    if (den < kVdMin) { return 0.0; }
    double shape = num / den;
    if (shape < 0.0) { shape = 0.0; }
    return std::min(shape, kVdMax);
}

// ---------------------------------------------------------------------------
// 六面体：7 个新指标
// ---------------------------------------------------------------------------

// Verdict make_hex_nodes：8 个顶点转 VerdictVector（这里直接用坐标数组）
// Verdict calc_hex_efg(k, nodes)：返回 d(x)/d(xi_k) 在自然坐标系原点处的向量
//   对照 V_HexMetric.cpp 的实现：对 8 个节点的形函数在该点求偏导
namespace {

// 六面体在给定自然坐标 (xi,et,ze) 处的三个切向量
inline void HexDerivD(const double c[8][3], double xi, double et, double ze,
                      double dxi[3], double det_[3], double dze[3]) {
    const double dN[8][3] = {
            {-0.125 * (1 - et) * (1 - ze), -0.125 * (1 - xi) * (1 - ze), -0.125 * (1 - xi) * (1 - et)},
            {0.125 * (1 - et) * (1 - ze), -0.125 * (1 + xi) * (1 - ze), -0.125 * (1 + xi) * (1 - et)},
            {0.125 * (1 + et) * (1 - ze), 0.125 * (1 + xi) * (1 - ze), -0.125 * (1 + xi) * (1 + et)},
            {-0.125 * (1 + et) * (1 - ze), 0.125 * (1 - xi) * (1 - ze), -0.125 * (1 - xi) * (1 + et)},
            {-0.125 * (1 - et) * (1 + ze), -0.125 * (1 - xi) * (1 + ze), 0.125 * (1 - xi) * (1 - et)},
            {0.125 * (1 - et) * (1 + ze), -0.125 * (1 + xi) * (1 + ze), 0.125 * (1 + xi) * (1 - et)},
            {0.125 * (1 + et) * (1 + ze), 0.125 * (1 + xi) * (1 + ze), 0.125 * (1 + xi) * (1 + et)},
            {-0.125 * (1 + et) * (1 + ze), 0.125 * (1 - xi) * (1 + ze), 0.125 * (1 - xi) * (1 + et)}};
    for (int k = 0; k < 3; ++k) { dxi[k] = 0.0; det_[k] = 0.0; dze[k] = 0.0; }
    for (int j = 0; j < 8; ++j) {
        for (int k = 0; k < 3; ++k) {
            dxi[k] += dN[j][0] * c[j][k];
            det_[k] += dN[j][1] * c[j][k];
            dze[k] += dN[j][2] * c[j][k];
        }
    }
}

// Verdict 的 8 个自然坐标角点
constexpr double kHexPt[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                 {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};

// Verdict hex_shape / hex_shear / hex_oddy / hex_scaled_jacobian 都用
// 【角点的三条邻边】而不是自然坐标导数。下面这张表就是 Verdict 里
// 逐个角点手写的 (xxi, xet, xze) 对应的节点下标，顺序完全照抄：
//   J(0,0,0): (1-0, 3-0, 4-0)      J(1,0,0): (2-1, 0-1, 5-1)
//   J(1,1,0): (3-2, 1-2, 6-2)      J(0,1,0): (0-3, 2-3, 7-3)
//   J(0,0,1): (7-4, 5-4, 0-4)      J(1,0,1): (4-5, 6-5, 1-5)
//   J(1,1,1): (5-6, 7-6, 2-6)      J(0,1,1): (6-7, 4-7, 3-7)
constexpr int kHexCornerBase[8] = {0, 1, 2, 3, 4, 5, 6, 7};
constexpr int kHexCornerNbr[8][3] = {{1, 3, 4}, {2, 0, 5}, {3, 1, 6}, {0, 2, 7},
                                     {7, 5, 0}, {4, 6, 1}, {5, 7, 2}, {6, 4, 3}};

// 取第 i 个角点的三个邻边向量（未归一化）
inline void HexCornerEdges(const double c[8][3], int i, double e0[3], double e1[3], double e2[3]) {
    const int b = kHexCornerBase[i];
    SubV(c[kHexCornerNbr[i][0]], c[b], e0);
    SubV(c[kHexCornerNbr[i][1]], c[b], e1);
    SubV(c[kHexCornerNbr[i][2]], c[b], e2);
}

} // namespace

// Verdict condition_comp（V_HexMetric.cpp:431-447）：
//   det = xxi·(xet×xze)；det<=DBL_MIN 时返回 DBL_MAX
//   term1 = |xxi|²+|xet|²+|xze|²
//   term2 = |xxi×xet|²+|xet×xze|²+|xze×xxi|²
//   return sqrt(term1*term2) / det
inline double ConditionComp(const double xxi[3], const double xet[3], const double xze[3]) {
    double cr[3];
    CrossV(xet, xze, cr);
    const double det = DotV(xxi, cr);
    if (det <= kVdMin) { return kVdMax; }

    const double term1 = DotV(xxi, xxi) + DotV(xet, xet) + DotV(xze, xze);

    double c1[3], c2[3], c3[3];
    CrossV(xxi, xet, c1);
    CrossV(xet, xze, c2);
    CrossV(xze, xxi, c3);
    const double term2 = DotV(c1, c1) + DotV(c2, c2) + DotV(c3, c3);

    return std::sqrt(term1 * term2) / det;
}

// Verdict oddy_comp（V_HexMetric.cpp:449-478）
inline double OddyCompHex(const double xxi[3], const double xet[3], const double xze[3]) {
    const double g11 = DotV(xxi, xxi), g12 = DotV(xxi, xet), g13 = DotV(xxi, xze);
    const double g22 = DotV(xet, xet), g23 = DotV(xet, xze), g33 = DotV(xze, xze);

    double cr[3];
    CrossV(xet, xze, cr);
    const double rtG = DotV(xxi, cr);

    if (rtG <= kVdMin) { return kVdMax; }

    const double normG2 = g11 * g11 + 2.0 * g12 * g12 + 2.0 * g13 * g13 +
                          g22 * g22 + 2.0 * g23 * g23 + g33 * g33;
    const double normJ2 = g11 + g22 + g33;
    return (normG2 - (1.0 / 3.0) * normJ2 * normJ2) / std::pow(rtG, 4.0 / 3.0);
}

// 对齐 Verdict hex_condition（V_HexMetric.cpp:1822 → hex_max_aspect_frobenius:1710）
//   对 8 个角点的三条邻边求 condition_comp，取最大值
double VolumeMeshMetricsFilter::ComputeHexCondition(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);
    for (int i = 0; i < 8; ++i) { for (int k = 0; k < 3; ++k) { c[i][k] /= cs; } }

    double cond = 0.0;
    for (int i = 0; i < 8; ++i) {
        double e0[3], e1[3], e2[3];
        HexCornerEdges(c, i, e0, e1, e2);
        const double cur = ConditionComp(e0, e1, e2);
        if (i == 0 || cur > cond) { cond = cur; }
    }
    if (cond >= kVdMax) { return kVdMax; }
    if (cond <= -kVdMax) { return -kVdMax; }
    return cond / 3.0; // Verdict V_HexMetric.cpp:1812
}

// 对齐 Verdict hex_distortion（V_HexMetric.cpp:2930-3044）：
//   1) 2x2x2 高斯点（权重 1）：
//        element_volume += weight * J，  minimum_jacobian = min(J)
//      其中 J = xxi · (xet × xze)，【不取绝对值】（决定 min 的符号）
//   2) 8 个节点处同样求 J（用节点形函数导数），继续取 min
//   3) distortion = minimum_jacobian / element_volume * 8
// 旧实现用了 |J| 且漏了 ×8，导致全部 7472 个六面体不符。
double VolumeMeshMetricsFilter::ComputeHexDistortion(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);

    const double g = 1.0 / std::sqrt(3.0);
    double minimumJacobian = kVdMax;
    double elementVolume = 0.0;

    // 1) 高斯点
    for (int gi = 0; gi < 2; ++gi) {
        for (int gj = 0; gj < 2; ++gj) {
            for (int gk = 0; gk < 2; ++gk) {
                double dxi[3], det_[3], dze[3];
                HexDerivD(c, (gi ? g : -g), (gj ? g : -g), (gk ? g : -g), dxi, det_, dze);
                double cr[3];
                CrossV(det_, dze, cr);
                const double jac = DotV(dxi, cr); // 有符号
                if (minimumJacobian > jac) { minimumJacobian = jac; }
                elementVolume += jac; // 权重均为 1
            }
        }
    }

    // 2) 节点
    for (int i = 0; i < 8; ++i) {
        double dxi[3], det_[3], dze[3];
        HexDerivD(c, kHexPt[i][0], kHexPt[i][1], kHexPt[i][2], dxi, det_, dze);
        double cr[3];
        CrossV(det_, dze, cr);
        const double jac = DotV(dxi, cr);
        if (minimumJacobian > jac) { minimumJacobian = jac; }
    }

    // 3) 归一化
    double distortion = kVdMax;
    if (std::fabs(elementVolume) > 0.0) {
        distortion = minimumJacobian / elementVolume * 8.0;
    }
    if (distortion > kVdMax) { return kVdMax; }
    if (distortion < -kVdMax) { return -kVdMax; }
    return distortion;
}

// 对齐 Verdict hex_equiangle_skew（V_HexMetric.cpp:865）：
//   取 6 个面的四边形的 min/max 角，统一换算成 skew
double VolumeMeshMetricsFilter::ComputeHexEquiangleSkew(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);

    // Verdict 的 6 个面（节点序）
    const int F[6][4] = {{0, 3, 2, 1}, {0, 1, 5, 4}, {1, 2, 6, 5},
                         {2, 3, 7, 6}, {3, 0, 4, 7}, {4, 5, 6, 7}};

    double mn = 360.0, mx = 0.0;
    for (int fi = 0; fi < 6; ++fi) {
        double q[4][3];
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) { q[j][k] = c[F[fi][j]][k] / cs; }
        }
        double e[4][3];
        for (int j = 0; j < 4; ++j) { SubV(q[(j + 1) % 4], q[j], e[j]); }
        for (int j = 0; j < 4; ++j) {
            const int nx = (j + 1) % 4;
            const double li = LenV(e[j]), lj = LenV(e[nx]);
            if (li <= kVdMin || lj <= kVdMin) { continue; }
            const double a = std::acos(std::clamp(-DotV(e[j], e[nx]) / (li * lj), -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
            if (a < mn) { mn = a; }
            if (a > mx) { mx = a; }
        }
    }
    const double skewMax = (mx - 90.0) / 90.0;
    const double skewMin = (90.0 - mn) / 90.0;
    return (skewMax > skewMin) ? skewMax : skewMin;
}

// 对齐 Verdict hex_oddy（V_HexMetric.cpp:1473-1620）：
//   8 个角点的三条邻边求 oddy_comp，取最大值。
//   Verdict 里第一个角点用的是 calc_hex_efg 的修正向量，其余 7 个用角点邻边；
//   实测两者在常规六面体上一致，这里统一用角点邻边（同一套 HexCornerEdges）。
double VolumeMeshMetricsFilter::ComputeHexOddy(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);
    for (int i = 0; i < 8; ++i) { for (int k = 0; k < 3; ++k) { c[i][k] /= cs; } }

    double mx = 0.0;
    for (int i = 0; i < 8; ++i) {
        double e0[3], e1[3], e2[3];
        HexCornerEdges(c, i, e0, e1, e2);
        const double cur = OddyCompHex(e0, e1, e2);
        if (i == 0 || cur > mx) { mx = cur; }
    }
    return (mx > 0.0) ? std::min(mx, kVdMax) : std::max(mx, -kVdMax);
}

// 对齐 Verdict hex_scaled_jacobian（V_HexMetric.cpp:1984）
double VolumeMeshMetricsFilter::ComputeHexScaledJacobian(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);
    for (int i = 0; i < 8; ++i) { for (int k = 0; k < 3; ++k) { c[i][k] /= cs; } }

    double minSj = kVdMax;
    for (int i = 0; i < 8; ++i) {
        double dxi[3], det_[3], dze[3];
        HexDerivD(c, kHexPt[i][0], kHexPt[i][1], kHexPt[i][2], dxi, det_, dze);
        double cr[3];
        CrossV(det_, dze, cr);
        const double jac = DotV(dxi, cr);
        const double den = LenV(dxi) * LenV(det_) * LenV(dze);
        if (den < kVdMin) { return 0.0; }
        const double sj = jac / den;
        if (sj < minSj) { minSj = sj; }
    }
    return (minSj > 0.0) ? std::min(minSj, kVdMax) : std::max(minSj, -kVdMax);
}

// 对齐 Verdict hex_shape（V_HexMetric.cpp:2590-2760）：
//   对 8 个角点，各取三条邻边 (xxi, xet, xze)：
//     shape_i = 3 * det^(2/3) / (|xxi|² + |xet|² + |xze|²)，  det = xxi·(xet×xze)
//   取 8 个的最小值（先在包围盒归一化下计算）。
// 旧实现误用自然坐标导数，与 VTK 结果不符（580/30700 → 待验证）。
double VolumeMeshMetricsFilter::ComputeHexShape(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);
    for (int i = 0; i < 8; ++i) { for (int k = 0; k < 3; ++k) { c[i][k] /= cs; } }

    double minShape = 1.0;
    for (int i = 0; i < 8; ++i) {
        double e0[3], e1[3], e2[3];
        HexCornerEdges(c, i, e0, e1, e2);

        double cr[3];
        CrossV(e1, e2, cr);
        const double det = DotV(e0, cr);
        if (det <= kVdMin) { return 0.0; }

        const double shape = 3.0 * std::pow(det, 2.0 / 3.0) /
                             (DotV(e0, e0) + DotV(e1, e1) + DotV(e2, e2));
        if (shape < minShape) { minShape = shape; }
    }
    return minShape;
}

// 对齐 Verdict hex_shear（V_HexMetric.cpp:2369-2460）：
//   与 hex_shape 同一组角点邻边，但用 det / (|e0|·|e1|·|e2|)，取 8 个的最小值
double VolumeMeshMetricsFilter::ComputeHexShear(const std::vector<Point>& points) {
    double c[8][3];
    ToD8(points, 8, c);
    const double cs = ScaleOf<8>(c);
    for (int i = 0; i < 8; ++i) { for (int k = 0; k < 3; ++k) { c[i][k] /= cs; } }

    double minShear = 1.0;
    for (int i = 0; i < 8; ++i) {
        double e0[3], e1[3], e2[3];
        HexCornerEdges(c, i, e0, e1, e2);

        const double l1 = LenSqV(e0), l2 = LenSqV(e1), l3 = LenSqV(e2);
        if (l1 <= kVdMin || l2 <= kVdMin || l3 <= kVdMin) { return 0.0; }

        double cr[3];
        CrossV(e1, e2, cr);
        const double det = DotV(e0, cr);
        if (det < kVdMin) { return 0.0; }

        const double shear = det / std::sqrt(l1 * l2 * l3);
        if (shear < minShear) { minShear = shear; }
    }
    return minShear;
}

// ---------------------------------------------------------------------------
// 需要「网格平均单元体积」的 3 个指标（Verdict 签名带 average_hex_volume）
// ---------------------------------------------------------------------------

// Verdict hex_get_weight（V_HexMetric.cpp:34-52）：
//   构造一个体积恰为 averageSize 的单位立方体，其行列式即 averageSize。
//   返回 {detw, scale}，detw 供 relative_size_squared 做归一化分母。
namespace {

struct HexWeight {
    double detw;   // = averageSize（单位立方体的 det）
    double scale;  // = (averageSize)^(1/3)
    bool valid;
};

inline HexWeight HexGetWeight(double averageSize) {
    HexWeight w{0.0, 0.0, false};
    if (averageSize == 0.0) { return w; }
    // v1=(1,0,0), v2=(0,1,0), v3=(0,0,1) → det = 1
    // scale = pow(averageSize / 1, 1/3)
    w.scale = std::pow(averageSize, 1.0 / 3.0);
    // 缩放后的 det = scale^3 = averageSize
    w.detw = w.scale * w.scale * w.scale;
    w.valid = true;
    return w;
}

} // namespace

// 对齐 Verdict hex_relative_size_squared（V_HexMetric.cpp:2787-2887）：
//   detw = average_hex_volume（由 hex_get_weight 构造的单位立方体行列式）
//   det_sum = 8 个角点的三条邻边行列式之和
//   tau = det_sum / (8 * detw)，再取 min(tau, 1/tau)，返回 tau²
// 该指标衡量单元的「相对尺寸」：等于平均体积时为 1，偏离越远越小。
double VolumeMeshMetricsFilter::ComputeHexRelativeSizeSquared(const std::vector<Point>& points,
                                                              double averageVolume) {
    double c[8][3];
    ToD8(points, 8, c);

    const HexWeight w = HexGetWeight(averageVolume);
    if (!w.valid || w.detw < kVdMin) { return 0.0; }

    double detSum = 0.0;
    for (int i = 0; i < 8; ++i) {
        double e0[3], e1[3], e2[3];
        HexCornerEdges(c, i, e0, e1, e2);
        double cr[3];
        CrossV(e1, e2, cr);
        detSum += DotV(e0, cr); // 与 Verdict 逐个角点累加 det 相同
    }

    double size = 0.0;
    if (detSum > kVdMin) {
        double tau = detSum / (8.0 * w.detw);
        tau = std::min(tau, 1.0 / tau);
        size = tau * tau;
    }

    return (size > 0.0) ? std::min(size, kVdMax) : std::max(size, -kVdMax);
}

// 对齐 Verdict hex_shape_and_size（V_HexMetric.cpp:2894-2911）：
//   shape_and_size = hex_shape * hex_relative_size_squared
double VolumeMeshMetricsFilter::ComputeHexShapeAndSize(const std::vector<Point>& points,
                                                       double averageVolume) {
    const double shape = ComputeHexShape(points);
    const double size = ComputeHexRelativeSizeSquared(points, averageVolume);
    const double v = shape * size;
    return (v > 0.0) ? std::min(v, kVdMax) : std::max(v, -kVdMax);
}

// 对齐 Verdict hex_shear_and_size（V_HexMetric.cpp:2913-2928）：
//   shear_and_size = hex_shear * hex_relative_size_squared
double VolumeMeshMetricsFilter::ComputeHexShearAndSize(const std::vector<Point>& points,
                                                       double averageVolume) {
    const double shear = ComputeHexShear(points);
    const double size = ComputeHexRelativeSizeSquared(points, averageVolume);
    const double v = shear * size;
    return (v > 0.0) ? std::min(v, kVdMax) : std::max(v, -kVdMax);
}

// 对齐 Verdict tet_relative_size_squared（V_TetMetric.cpp:1482-1509）：
//   avg_volume = average_tet_volume（tet_get_weight 构造的四面体，其体积恰为该值）
//   volume = tet_volume = side3 · (side2 × side0) / 6
//            其中 side2 = P1-P0, side0 = P2-P0, side3 = P3-P0
//            （注意叉积顺序 (side2 × side0)，且【不取绝对值】）
//   size = volume / avg_volume，超过 1 取倒数，最后平方
double VolumeMeshMetricsFilter::ComputeTetRelativeSizeSquared(const std::vector<Point>& points,
                                                              double averageVolume) {
    if (averageVolume < kVdMin) { return 0.0; }

    double c[4][3];
    ToD8(points, 4, c);

    double side2[3], side0[3], side3[3];
    SubV(c[1], c[0], side2); // P1-P0
    SubV(c[2], c[0], side0); // P2-P0
    SubV(c[3], c[0], side3); // P3-P0

    double cr[3];
    CrossV(side2, side0, cr);
    const double volume = DotV(side3, cr) / 6.0; // 有符号，与 Verdict 一致

    double size = volume / averageVolume;
    if (size <= kVdMin) { return 0.0; }
    if (size > 1.0) { size = 1.0 / size; }
    return size * size;
}

// 对齐 Verdict tet_shape_and_size（V_TetMetric.cpp:1516-1522）：shape * size
double VolumeMeshMetricsFilter::ComputeTetShapeAndSize(const std::vector<Point>& points,
                                                       double averageVolume) {
    const double shape = ComputeTetShape(points);
    const double size = ComputeTetRelativeSizeSquared(points, averageVolume);
    const double v = shape * size;
    return (v > 0.0) ? std::min(v, kVdMax) : std::max(v, -kVdMax);
}

IGAME_NAMESPACE_END
