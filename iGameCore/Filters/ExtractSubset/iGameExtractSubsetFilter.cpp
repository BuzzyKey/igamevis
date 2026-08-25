#include "ExtractSubset/iGameExtractSubsetFilter.h"
#include "iGamePoints.h"
#include "iGameCellArray.h"

IGAME_NAMESPACE_BEGIN

bool ExtractSubsetFilter::Execute()
{
    UpdateProgress(0);

    m_InputMesh = DynamicCast<StructuredMesh>(GetInput(0));
    if (m_InputMesh == nullptr) {
        return false;
    }

    igIndex* inputSize = m_InputMesh->GetDimensionSize();

    int minI = m_VOI[0];
    int maxI = m_VOI[1];
    int minJ = m_VOI[2];
    int maxJ = m_VOI[3];
    int minK = m_VOI[4];
    int maxK = m_VOI[5];

    if (maxI < 0) maxI = inputSize[0] - 1;
    if (maxJ < 0) maxJ = inputSize[1] - 1;
    if (maxK < 0) maxK = inputSize[2] - 1;

    if (minI < 0 || minI >= inputSize[0] ||
        maxI < 0 || maxI >= inputSize[0] ||
        minJ < 0 || minJ >= inputSize[1] ||
        maxJ < 0 || maxJ >= inputSize[1] ||
        minK < 0 || minK >= inputSize[2] ||
        maxK < 0 || maxK >= inputSize[2]) {
        return false;
    }

    if (minI > maxI || minJ > maxJ || minK > maxK) {
        return false;
    }

    m_OutputMesh = StructuredMesh::New();

    igIndex newSize[3];
    newSize[0] = maxI - minI + 1;
    newSize[1] = maxJ - minJ + 1;
    newSize[2] = maxK - minK + 1;

    m_OutputMesh->SetDimensionSize(newSize);

    Points::Pointer outputPoints = Points::New();
    outputPoints->Resize(newSize[0] * newSize[1] * newSize[2]);

    igIndex ptIdx = 0;

    for (igIndex k = minK; k <= maxK; ++k) {
        for (igIndex j = minJ; j <= maxJ; ++j) {
            for (igIndex i = minI; i <= maxI; ++i) {
                igIndex oldPtId = m_InputMesh->GetPointIndex(i, j, k);
                Point p = m_InputMesh->GetPoint(oldPtId);
                outputPoints->SetPoint(ptIdx++, p);
            }
        }
    }

    m_OutputMesh->SetPoints(outputPoints);
    UpdateProgress(0.3);

    if (newSize[2] > 1) {
        CellArray::Pointer volumes = CellArray::New();

        igIndex vhs[8];
        igIndex tmpvhs[8] = {0,
                             1,
                             1 + newSize[0],
                             newSize[0],
                             newSize[0] * newSize[1],
                             1 + newSize[0] * newSize[1],
                             1 + newSize[0] + newSize[0] * newSize[1],
                             newSize[0] + newSize[0] * newSize[1]};

        for (igIndex k = 0; k < newSize[2] - 1; ++k) {
            for (igIndex j = 0; j < newSize[1] - 1; ++j) {
                igIndex st = j * newSize[0] + k * newSize[0] * newSize[1];
                for (igIndex i = 0; i < newSize[0] - 1; ++i) {
                    for (int it = 0; it < 8; ++it) {
                        vhs[it] = st + tmpvhs[it];
                    }
                    volumes->AddCellIds(vhs, 8);
                    ++st;
                }
            }
        }

        m_OutputMesh->SetVolumes(volumes);
        m_OutputMesh->GenStructuredCellConnectivities();
        UpdateProgress(0.7);
    } else if (newSize[1] > 1) {
        CellArray::Pointer faces = CellArray::New();

        igIndex fhs[4];
        igIndex tmpfhs[4] = {0, 1, newSize[0] + 1, newSize[0]};

        for (igIndex j = 0; j < newSize[1] - 1; ++j) {
            igIndex st = j * newSize[0];
            for (igIndex i = 0; i < newSize[0] - 1; ++i) {
                for (int it = 0; it < 4; ++it) {
                    fhs[it] = st + tmpfhs[it];
                }
                faces->AddCellIds(fhs, 4);
                ++st;
            }
        }

        m_OutputMesh->SetFaces(faces);
        m_OutputMesh->BuildStructuredFaces();
        UpdateProgress(0.7);
    }

    UpdateProgress(0.9);

    UpdateProgress(1.0);

    SetOutput(0, m_OutputMesh);
    return true;
}

IGAME_NAMESPACE_END
