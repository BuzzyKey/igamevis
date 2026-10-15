#include "Convert/iGameConvertToSurfaceMeshFilter.h"
#include "iGameSurfaceMeshMetricsFilter.h"
#include <string>

IGAME_NAMESPACE_BEGIN

SurfaceMeshMetricsFilter::SurfaceMeshMetricsFilter() {
    this->SetNumberOfInputs(1);
    this->SetNumberOfOutputs(1);
    m_Faces = nullptr;
    m_Points = nullptr;
}

SurfaceMeshMetricsFilter::~SurfaceMeshMetricsFilter() {
    m_Faces = nullptr;
    m_Points = nullptr;
}


bool SurfaceMeshMetricsFilter::Execute() {
    if (m_Inputs->GetNumberOfElements() == 0) { return false; }

    auto input = m_Inputs->GetElement(0);
    if (!input) { return false; }
    m_Faces = nullptr;
    switch (input->GetDataObjectType()) {
        case IG_SURFACE_MESH:
            m_Faces = (DynamicCast<SurfaceMesh>(input))->GetFaces();
            m_Points = (DynamicCast<SurfaceMesh>(input))->GetPoints();
            break;
        case IG_UNSTRUCTURED_MESH: {
            auto mesh = DynamicCast<UnstructuredMesh>(input);
            auto converter = ConvertToSurfaceMeshFilter::New();
            converter->SetInput(mesh);
            bool result = converter->Execute();
            if (result) {
                m_Faces = converter->GetSurfaceMesh()->GetFaces();
                m_Points = converter->GetSurfaceMesh()->GetPoints();
            }
            break;
        }
        default:
            igDebug("请输入表面网格进行质量检测");
            break;
    }
    if (!m_Faces) {
        igDebug("没有表面单元");
        return false;
    }
    igIndex vhs[IGAME_CELL_MAX_SIZE] = {0}; //存储每个面的顶点索引数组
    igIndex vNum = 0;                       //每个面的顶点数量

    igIndex faceNum = m_Faces->GetNumberOfCells(); //总面数

    DoubleArray::Pointer metricArray = DoubleArray::New();
    metricArray->SetName(std::string("Metric") + std::to_string(m_Metric));

    metricArray->SetDimension(1);
    metricArray->Reserve(faceNum);
    DoubleArray::Pointer dataRange = DoubleArray::New();
    dataRange->AddValue(0.0);
    dataRange->AddValue(1.0);
    dataRange->AddValue(0.0);
    dataRange->AddValue(1.0);
    for (igIndex i = 0; i < faceNum; i++) {
        vNum = m_Faces->GetCellIds(i, vhs); //获取第i个面的顶点索引
        double metric = this->ComputeMetric(vNum, vhs);
        metricArray->AddValue(metric);
    }

    auto output = input;

    output->GetAttributeSet()->AddAttribute(IG_SCALAR, IG_CELL, metricArray);
    //output->GetAttributeSet()->AddAttribute(IG_SCALAR,IG_CELL,metricArray,dataRange);

    this->SetOutput(output);

    return true;
}


double SurfaceMeshMetricsFilter::ComputeMetric(igIndex vNum, igIndex* vhs) {
    std::vector<Point> points;
    for (igIndex i = 0; i < vNum; i++) { points.push_back(m_Points->GetPoint(vhs[i])); }
    //Point p[100];
    //for (igIndex i = 0; i < vNum; i++) {
    //	p[i]=m_Points->GetPoint(vhs[0]);
    //}
    //double len= (p[1]-p[0]).norm();
    //return ComputeA(vNum, vhs);

    // 根据顶点数量选择计算方法
    if (vNum == 3) {
        // 三角形
        switch (m_Metric) {
            case FACE_AREA:
                return ComputeTriangleArea(points);
                break;
            case MAX_ANGLE:
                return ComputeTriangleMaxAngle(points);
                break;
            case MIN_ANGLE:
                return ComputeTriangleMinAngle(points);
                break;
            case JACOBIAN:
                return ComputeTriangleJacobian(points);
                break;
            case ASPECT_RATIO:
                return ComputeTriangleAspectRatio(points);
                break;
            case EDGE_RATIO:
                return ComputeTriangleEdgeRatio(points);
                break;
            /*case AUTO_QUALITY:
                return ComputeTriangleMeshQuality(points);
                break;*/
            default:
                break;
        }
    } else if (vNum == 4) {
        // 四边形
        switch (m_Metric) {
            case FACE_AREA:
                return ComputeQuadArea(points);
                break;
            case MAX_ANGLE:
                return ComputeQuadMaxAngle(points);
                break;
            case MIN_ANGLE:
                return ComputeQuadMinAngle(points);
                break;
            case JACOBIAN:
                return ComputeQuadJacobian(points);
                break;
            case ASPECT_RATIO:
                return ComputeQuadAspectRatio(points);
                break;
            case EDGE_RATIO:
                return ComputeQuadEdgeRatio(points);
                break;
            case WARPAGE:
                return ComputeQuadWarpage(points);
                break;
            case TAPER:
                return ComputeQuadTaper(points);
                break;
            case SKEW:
                return ComputeQuadSkew(points);
                break;
            //case AUTO_QUALITY:
            //    return ComputeQuadSkew(points); // 默认用歪斜度
            //    break;
            default:
                break;
        }
    } else {
    }

    //switch (m_Metric)
    //{
    //   case TRIANGLE_AREA:
    //       return ComputeTriangleArea(points);
    //	break;
    //   case MAX_ANGLE:
    //       return ComputeMaxAngle(points);
    //	break;
    //default:
    //	break;
    //}
    return 0.0;
}
//计算顶点内角
double SurfaceMeshMetricsFilter::GetInternalAnglesOfVertex(Point v0, Point v1, Point v2) {
    Point e1 = v1 - v0;
    Point e2 = v2 - v0;
    double denom = e1.norm() * e2.norm();
    if (denom < 1e-12) return 0.0;
    double cosa = e1.dot(e2) / denom;
    cosa = std::clamp(cosa, -1.0, 1.0);
    double angle = acos(cosa) * 180.0 / PI;
    return angle;
}


// 三角形_面积
double SurfaceMeshMetricsFilter::ComputeTriangleArea(const std::vector<Point>& points) {
    Point edge1 = points[1] - points[0];
    Point edge2 = points[2] - points[0];
    Point cross = edge1.cross(edge2);
    return cross.norm() / 2.0;
}


double SurfaceMeshMetricsFilter::ComputeTriangleMaxAngle(const std::vector<Point>& points) {

    double angle1 = GetInternalAnglesOfVertex(points[0], points[1], points[2]);
    double angle2 = GetInternalAnglesOfVertex(points[1], points[0], points[2]);
    double angle3 = GetInternalAnglesOfVertex(points[2], points[1], points[0]);

    return std::max({angle1, angle2, angle3});
}

// 三角形最小内角（度）
// 严格对齐 Verdict tri_minimum_angle（V_TriMetric.cpp:422-483）：
//   1) 三条边向量 sides[0]=P1-P0, sides[1]=P2-P1, sides[2]=P2-P0, sides[3]=-sides[1]
//   2) 找出最短边 short_side
//   3) 只计算该边所对的角（最短边对最小角），用 acos(dot/(|a||b|))
// 旧实现分别算 3 个角再取 min：数学上等价，但 acos 在接近 1 处导数极大，
// 三次调用中任意一次舍入都会放大误差（实测 bicycle 上 4/96668 个单元超差）。
// 全程 double 以避免 Vector3f 的抵消误差。
double SurfaceMeshMetricsFilter::ComputeTriangleMinAngle(const std::vector<Point>& points) {
    auto sub = [&](int a, int b, double out[3]) {
        out[0] = static_cast<double>(points[a][0]) - points[b][0];
        out[1] = static_cast<double>(points[a][1]) - points[b][1];
        out[2] = static_cast<double>(points[a][2]) - points[b][2];
    };
    auto dot = [](const double u[3], const double v[3]) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };
    auto sq = [](const double v[3]) { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; };

    double sides[4][3];
    sub(1, 0, sides[0]);   // P1 - P0
    sub(2, 1, sides[1]);   // P2 - P1
    sub(2, 0, sides[2]);   // P2 - P0
    for (int k = 0; k < 3; ++k) { sides[3][k] = -sides[1][k]; }

    const double l0 = sq(sides[0]), l1 = sq(sides[1]), l2 = sq(sides[2]);
    if (l0 == 0.0 || l1 == 0.0 || l2 == 0.0) { return 0.0; }

    int shortSide = 0;
    if (l1 < l0) { shortSide = 1; }
    if (l2 < (shortSide == 0 ? l0 : l1)) { shortSide = 2; }

    const double* a = nullptr;
    const double* b = nullptr;
    if (shortSide == 0) { a = sides[2]; b = sides[1]; }
    else if (shortSide == 1) { a = sides[0]; b = sides[2]; }
    else { a = sides[0]; b = sides[3]; }

    const double la = std::sqrt(sq(a));
    const double lb = std::sqrt(sq(b));
    if (la <= 0.0 || lb <= 0.0) { return 0.0; }

    const double cosA = std::clamp(dot(a, b) / (la * lb), -1.0, 1.0);
    return std::acos(cosA) * 180.0 / PI;
}

// 雅可比, range : [0, MAX], acceptable range [0, MAX], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeTriangleJacobian(const std::vector<Point>& points) {
    //be caution about order
    Point edge0 = points[1] - points[0];
    Point edge1 = points[2] - points[0];
    Point edge2 = points[2] - points[1];

    Point first = edge1 - edge0;
    Point second = edge2 - edge0;

    Point cross = first.cross(second);
    double jacobian = cross.norm();

    double max_edge_length_product;
    max_edge_length_product =
            std::max({edge0.norm() * edge1.norm(), edge1.norm() * edge2.norm(), edge0.norm() * edge2.norm()});


    jacobian *= (2.0 / sqrt(3.0));
    jacobian /= max_edge_length_product;

    return jacobian;
}

// 纵横比 , range : [1, MAX], acceptable range [1, 1.3], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeTriangleAspectRatio(const std::vector<Point>& points) {
    double a = (points[1] - points[0]).norm();
    double b = (points[2] - points[1]).norm();
    double c = (points[0] - points[2]).norm();
    double area = ComputeTriangleArea(points);

    return (std::max({a, b, c})) * (a + b + c) / (4.0 * sqrt(3.0) * area);
}

//长宽比
double SurfaceMeshMetricsFilter::ComputeTriangleEdgeRatio(const std::vector<Point>& points) {
    double a = (points[1] - points[0]).norm();
    double b = (points[2] - points[1]).norm();
    double c = (points[0] - points[2]).norm();

    return std::max({a, b, c}) / std::min({a, b, c});
}

// 网格质量, range : [0, 1], acceptable range [0.8, 1], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeTriangleMeshQuality(const std::vector<Point>& points) {
    double area = ComputeTriangleArea(points);
    double a = (points[1] - points[0]).norm();
    double b = (points[2] - points[1]).norm();
    double c = (points[0] - points[2]).norm();

    double in_r = 2.0 * area / (a + b + c);
    double h = std::max({a, b, c});

    //double Q = 2 * sqrt(3) *area/ (in_r * h); //这个最大是3根号3啊，论文里面写错了
    double Q = 2.0 * sqrt(3.0) * area / ((a + b + c) * 0.5 * h);
    //double Q = 4 * sqrt(3) * area / (a * a + b * b + c * c);

    return Q;
}


// ==================== 四边形计算方法 ====================

// 面积 range : [0, MAX], acceptable range [0, MAX], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeQuadArea(const std::vector<Point>& points) {
    double area1 = (points[1] - points[0]).cross(points[2] - points[0]).norm() / 2.0;
    double area2 = (points[2] - points[0]).cross(points[3] - points[0]).norm() / 2.0;
    double area = area1 + area2;
    return area;
}

// 最大角, range : [0, π], acceptable range [π/2, π3/4], unit cube : π/3
double SurfaceMeshMetricsFilter::ComputeQuadMaxAngle(const std::vector<Point>& points) {
    std::vector<double> angles;
    angles.push_back(GetInternalAnglesOfVertex(points[1], points[0], points[2]));
    angles.push_back(GetInternalAnglesOfVertex(points[2], points[1], points[3]));
    angles.push_back(GetInternalAnglesOfVertex(points[3], points[2], points[0]));
    angles.push_back(GetInternalAnglesOfVertex(points[0], points[3], points[1]));

    return *std::max_element(angles.begin(), angles.end());
}

// 最小角, range : [0, π]
// 对齐 Verdict quad_minimum_angle（V_QuadMetric.cpp:981-1036）：
//   顶点 i 处的内角 = acos( -(e[i-1] · e[i]) / (|e[i-1]| * |e[i]|) )。
// 全程 double：Vector3f 的抵消误差会让极小角偏差一个量级以上
// （实测 bicycle 上 5/96668 个单元不符）。
double SurfaceMeshMetricsFilter::ComputeQuadMinAngle(const std::vector<Point>& points) {
    auto sub = [&](int a, int b, double out[3]) {
        out[0] = static_cast<double>(points[a][0]) - points[b][0];
        out[1] = static_cast<double>(points[a][1]) - points[b][1];
        out[2] = static_cast<double>(points[a][2]) - points[b][2];
    };
    double e[4][3];
    sub(1, 0, e[0]);
    sub(2, 1, e[1]);
    sub(3, 2, e[2]);
    sub(0, 3, e[3]);

    double mn = 360.0;
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        const double li = std::sqrt(e[i][0] * e[i][0] + e[i][1] * e[i][1] + e[i][2] * e[i][2]);
        const double lj = std::sqrt(e[j][0] * e[j][0] + e[j][1] * e[j][1] + e[j][2] * e[j][2]);
        if (li <= 0.0 || lj <= 0.0) { continue; }
        const double d = e[i][0] * e[j][0] + e[i][1] * e[j][1] + e[i][2] * e[j][2];
        const double angle = std::acos(std::clamp(-d / (li * lj), -1.0, 1.0)) * 180.0 / PI;
        if (angle < mn) { mn = angle; }
    }
    return mn;
}

// 雅可比, range : [0, MAX], acceptable range [0, MAX], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeQuadJacobian(const std::vector<Point>& points) {
    // 对齐 Verdict quad_jacobian（V_QuadMetric.cpp:1169-1186）：
    //   jacobian = min(0..3) of [ 中心法向 · 各顶点角法向 ]
    //   顶点角法向是【未归一化】的叉积（等于 2 倍三角形面积）。
    // 旧实现对角法向多做了 .normalized()，于是平面四边形的结果恒为 ±1，
    // 丢了面积量纲：2x1 矩形应得 2.0，旧实现得 1.0。
    const Point e0 = points[1] - points[0];
    const Point e1 = points[2] - points[1];
    const Point e2 = points[3] - points[2];
    const Point e3 = points[0] - points[3];

    const Point cn0 = e3.cross(e0);
    const Point cn1 = e0.cross(e1);
    const Point cn2 = e1.cross(e2);
    const Point cn3 = e2.cross(e3);

    const Point pa0 = e0 - e2;
    const Point pa1 = e1 - e3;
    Point n_c = pa0.cross(pa1);
    const double nLen = n_c.norm();
    if (nLen < 1e-30) { return 0.0; } // 退化四边形
    n_c = n_c / nLen;

    const double a0 = n_c.dot(cn0);
    const double a1 = n_c.dot(cn1);
    const double a2 = n_c.dot(cn2);
    const double a3 = n_c.dot(cn3);

    return std::min(std::min(a0, a1), std::min(a2, a3));
}

// 纵横比 , range : [1, MAX]
// 严格对齐 Verdict quad_aspect_ratio（V_QuadMetric.cpp:448-479）：
//   hm = max(4 条边长)
//   分母 = Σ assert_signed_corner_areas[i]（4 个顶点的【带符号角面积】之和）
//   aspect_ratio = hm * (a1+b1+c1+d1) / 分母
// 关键：分母不是 4 倍面积，而是 signed_corner_areas 之和 ——
// 两者对平面四边形数值接近但不相等（面积用三角形分解，角面积用顶点叉积），
// 这正是旧实现偏差 1.7e-4 ~ 2.8e-3、15736/96668 个单元超差的原因。
// 全程 double，且 char_size 归一化在比值中抵消，此处直接在原始尺度计算。
double SurfaceMeshMetricsFilter::ComputeQuadAspectRatio(const std::vector<Point>& points) {
    auto sub = [&](int a, int b, double out[3]) {
        out[0] = static_cast<double>(points[a][0]) - points[b][0];
        out[1] = static_cast<double>(points[a][1]) - points[b][1];
        out[2] = static_cast<double>(points[a][2]) - points[b][2];
    };
    auto len = [](const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };
    auto cross = [](const double u[3], const double v[3], double out[3]) {
        out[0] = u[1] * v[2] - u[2] * v[1];
        out[1] = u[2] * v[0] - u[0] * v[2];
        out[2] = u[0] * v[1] - u[1] * v[0];
    };
    auto dot = [](const double u[3], const double v[3]) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };

    // 4 条边（Verdict make_quad_edges 的同一约定）
    double e[4][3];
    sub(1, 0, e[0]);
    sub(2, 1, e[1]);
    sub(3, 2, e[2]);
    sub(0, 3, e[3]);

    const double l0 = len(e[0]), l1 = len(e[1]), l2 = len(e[2]), l3 = len(e[3]);
    double ma = (l0 > l1) ? l0 : l1;
    double mb = (l2 > l3) ? l2 : l3;
    const double hm = (ma > mb) ? ma : mb;

    // signed_corner_areas：中心法向 · 各顶点角法向（未归一化叉积）
    double cn[4][3];
    cross(e[3], e[0], cn[0]);
    cross(e[0], e[1], cn[1]);
    cross(e[1], e[2], cn[2]);
    cross(e[2], e[3], cn[3]);

    double pa0[3], pa1[3], nc[3];
    for (int k = 0; k < 3; ++k) { pa0[k] = e[0][k] - e[2][k]; }
    for (int k = 0; k < 3; ++k) { pa1[k] = e[1][k] - e[3][k]; }
    cross(pa0, pa1, nc);
    const double nlen = len(nc);
    if (nlen <= 0.0) { return 1e30; }
    for (int k = 0; k < 3; ++k) { nc[k] /= nlen; }

    const double denom = dot(nc, cn[0]) + dot(nc, cn[1]) + dot(nc, cn[2]) + dot(nc, cn[3]);
    if (std::fabs(denom) < 1e-300) { return 1e30; }

    const double aspectRatio = hm * (l0 + l1 + l2 + l3) / denom;
    return (aspectRatio < 1e30) ? aspectRatio : 1e30;
}

// 长宽比 , range : [1, MAX], acceptable range [1, 1.3], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeQuadEdgeRatio(const std::vector<Point>& points) {
    std::vector<double> v;
    v.push_back((points[1] - points[0]).norm());
    v.push_back((points[2] - points[1]).norm());
    v.push_back((points[3] - points[2]).norm());
    v.push_back((points[0] - points[3]).norm());

    return *std::max_element(v.begin(), v.end()) / *std::min_element(v.begin(), v.end());
}

// 伸展度 (Stretch), range : [0, 1]
// 对齐 Verdict quad_stretch（V_QuadMetric.cpp:864-903）：
//   1) 特征尺寸 char_size = 8 个坐标包围盒的对角线长度，用它归一化 4 条边与 2 条对角线；
//   2) diag = max(|P2-P0|², |P3-P1|²) / char_size²；
//   3) stretch = sqrt(2) * sqrt( min(4 条边长²) / diag )。
// 注：这是一次比值运算，char_size 在分子分母中会部分抵消，
//     但 Verdict 的实现确实先做了归一化，为求逐值一致这里照做。
double SurfaceMeshMetricsFilter::ComputeQuadStretch(const std::vector<Point>& points) {
    // 包围盒对角线（Verdict elem_scaling）
    Point bmin = points[0];
    Point bmax = points[0];
    for (int i = 1; i < 4; ++i) {
        for (int k = 0; k < 3; ++k) {
            if (points[i][k] < bmin[k]) { bmin[k] = points[i][k]; }
            if (points[i][k] > bmax[k]) { bmax[k] = points[i][k]; }
        }
    }
    double charSize = (bmax - bmin).norm();
    if (charSize < 1e-30) { charSize = 1.0; }

    const double inv = 1.0 / charSize;
    const Point e0 = (points[1] - points[0]) * inv;
    const Point e1 = (points[2] - points[1]) * inv;
    const Point e2 = (points[3] - points[2]) * inv;
    const Point e3 = (points[0] - points[3]) * inv;

    const double l0 = e0.squaredNorm();
    const double l1 = e1.squaredNorm();
    const double l2 = e2.squaredNorm();
    const double l3 = e3.squaredNorm();

    const double d02 = ((points[2] - points[0]) * inv).squaredNorm();
    const double d13 = ((points[3] - points[1]) * inv).squaredNorm();
    const double diag = (d02 > d13) ? d02 : d13;

    if (diag < 1e-300) { return 0.0; }

    double minLen2 = (l0 < l1) ? l0 : l1;
    const double m23 = (l2 < l3) ? l2 : l3;
    if (m23 < minLen2) { minLen2 = m23; }

    return std::sqrt(2.0) * std::sqrt(minLen2 / diag);
}

// 翘曲 , range : [0, 1], acceptable range [0, 0.7], unit cube : 0
double SurfaceMeshMetricsFilter::ComputeQuadWarpage(const std::vector<Point>& points) {

    Point n0 = ((points[0] - points[3]).cross(points[1] - points[0])).normalized();
    Point n1 = ((points[1] - points[0]).cross(points[2] - points[1])).normalized();
    Point n2 = ((points[2] - points[1]).cross(points[3] - points[2])).normalized();
    Point n3 = ((points[3] - points[2]).cross(points[0] - points[3])).normalized();

    double m1 = (n0 * n2) * (n0 * n2) * (n0 * n2);
    double m2 = (n1 * n3) * (n1 * n3) * (n1 * n3);

    return 1.0 - std::min(m1, m2);
}

// 锥度, range : [0, MAX], acceptable range [0, 0.7], unit cube : 0
double SurfaceMeshMetricsFilter::ComputeQuadTaper(const std::vector<Point>& points) {
    Point x_1 = (points[1] - points[0]) + (points[2] - points[3]);
    Point x_2 = (points[2] - points[1]) + (points[3] - points[0]);
    Point x_12 = (points[0] - points[1]) + (points[2] - points[3]);

    return x_12.norm() / std::min(x_1.norm(), x_2.norm());
}

// 歪斜度 , range : [0, 1], acceptable range [0.5, 1], unit cube : 1
double SurfaceMeshMetricsFilter::ComputeQuadSkew(const std::vector<Point>& points) {
    Point x_1 = (points[1] - points[0]) + (points[2] - points[3]);
    Point x_2 = (points[2] - points[1]) + (points[3] - points[0]);
    Point x_1_norm = x_1 / x_1.norm();
    Point x_2_norm = x_2 / x_2.norm();

    return std::abs(x_1_norm * x_2_norm);
}


/*
* 表面网格面片最小角质量计算
* 基于MeshMath.h中的get_min_angle_quality_SurfaceMesh
*/
double SurfaceMeshMetricsFilter::ComputeFaceMinAngleQuality(const std::vector<Point>& points) {
    if (points.size() < 3) { return 0.0; }

    double min_angle = 180.0;
    int n = points.size();

    // 遍历每个顶点计算内角
    for (int i = 0; i < n; ++i) {
        auto& v0 = points[i];
        auto& v1 = points[(i + 1) % n];
        auto& v2 = points[(i + 2) % n];

        auto vec10 = (v0 - v1).normalized();
        auto vec12 = (v2 - v1).normalized();
        double angle = std::acos(vec10.dot(vec12)) / M_PI * 180.0;
        min_angle = std::min(min_angle, angle);
    }

    // 根据面类型计算质量值
    double val = std::fmin(1.0, (n == 3) ? (min_angle / 60.0) : (min_angle / 90.0));
    return val;
}


/*
* 表面网格体积计算
* 基于MeshMath.h中的get_volume_surface_mesh
*/
double SurfaceMeshMetricsFilter::ComputeSurfaceVolume(const std::vector<Point>& points) {
    // 注意：这个函数通常用于整个表面网格，而不是单个面片
    // 这里实现单个面片的体积贡献计算

    if (points.size() < 3) { return 0.0; }

    double volume_contribution = 0.0;

    // 将多边形分解为三角形并计算体积贡献
    for (int i = 2; i < points.size(); ++i) {
        auto& v0 = points[0];
        auto& v1 = points[i - 1];
        auto& v2 = points[i];

        // 使用原始算法：v_tet = (v0 % v1) * v2
        double v_tet = (v0.cross(v1)).dot(v2);
        volume_contribution += v_tet;
    }

    return std::abs(volume_contribution) / 6.0;
}


/*
* 表面网格的体积
*/
double SurfaceMeshMetricsFilter::ComputeTotalSurfaceVolume(CellArray::Pointer m_Faces, Points::Pointer m_Points) {
    if (!m_Faces || !m_Points) { return 0.0; }

    double total_volume = 0.0;
    igIndex vhs[IGAME_CELL_MAX_SIZE] = {0};

    igIndex cellNum = m_Faces->GetNumberOfCells();

    for (igIndex i = 0; i < cellNum; i++) {
        igIndex vNum = m_Faces->GetCellIds(i, vhs);
        std::vector<Point> points;

        for (igIndex j = 0; j < vNum; j++) { points.push_back(m_Points->GetPoint(vhs[j])); }

        // 累加每个面片的体积贡献
        total_volume += ComputeSurfaceVolume(points);
    }

    return total_volume;
}


IGAME_NAMESPACE_END
