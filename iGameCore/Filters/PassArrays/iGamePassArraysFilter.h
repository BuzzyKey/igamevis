#ifndef iGamePassArraysFilter_h
#define iGamePassArraysFilter_h

#include "iGameDataObject.h"
#include "iGameFilter.h"
#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN

/**
 * 传递过滤数据数组 (Pass Arrays)
 *
 * 语义与 VTK 的 PassArrays 一致：输出对象与输入对象共享几何与拓扑（点坐标、单元连接关系等
 * 都按引用共享，不复制），只保留被选中的属性数组；被保留的数组本身同样按引用共享，
 * 不做深拷贝。
 *
 * 点数据 (Point Data) 与单元数据 (Cell Data) 的选择分别记录：
 * 当点数据和单元数据上存在同名属性（例如都叫 test）时，可以只保留其中一项。
 */
class PassArrays : public Filter {
public:
    I_OBJECT(PassArrays);
    static Pointer New() { return new PassArrays; }

    /* ---------- 点数据 (Point Data) 选择 ---------- */
    // 设置需要保留的点属性名称
    void SetPointArrayNames(const std::vector<std::string>& names);
    void AddPointArrayName(const std::string& name);
    const std::vector<std::string>& GetPointArrayNames() const { return m_PointArrayNames; }

    /* ---------- 单元数据 (Cell Data) 选择 ---------- */
    // 设置需要保留的单元属性名称
    void SetCellArrayNames(const std::vector<std::string>& names);
    void AddCellArrayName(const std::string& name);
    const std::vector<std::string>& GetCellArrayNames() const { return m_CellArrayNames; }

    // 同时设置点数据与单元数据的选择（两者分开保存，不会互相影响）
    void SetArrayNames(const std::vector<std::string>& pointNames,
                       const std::vector<std::string>& cellNames);

    // 清空所有选择（输出将不保留任何属性数组）
    void ClearArrayNames();

    bool Execute() override;

protected:
    PassArrays() {
        SetNumberOfInputs(1);
        SetNumberOfOutputs(1);
    }
    ~PassArrays() override = default;

private:
    std::vector<std::string> m_PointArrayNames;
    std::vector<std::string> m_CellArrayNames;
};

IGAME_NAMESPACE_END

#endif
