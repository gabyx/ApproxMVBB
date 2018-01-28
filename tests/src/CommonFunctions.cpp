// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "CommonFunctions.hpp"
#include <iomanip>
#include "TestConfig.hpp"

namespace ApproxMVBB
{
    namespace TestFunctions
    {
        std::size_t hashString(std::string s)
        {
            std::size_t h = 3141459;
            for(char& c : s)
            {
                h = h * 101 + c;
            }
            return h;
        }

        int isBigEndian(void)
        {
            union
            {
                uint32_t i;
                char c[4];
            } bint = {0x01020304};

            return bint.c[0] == 1;
        }

        template<>
        std::string getPrecAbrev<double>()
        {
            return "double";
        }
        template<>
        std::string getPrecAbrev<float>()
        {
            return "float";
        }

        std::string getFileInPath(std::string name)
        {
            return ApproxMVBB_TESTS_INPUT_FILES_DIR "/" + name;
        }

        std::string getFileInAddPath(std::string name)
        {
            return ApproxMVBB_TESTS_INPUT_FILES_ADD_DIR "/" + name;
        }

        std::string getPointsDumpPath(std::string name, std::string suffix)
        {
            return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + name + "-" + getPrecAbrev() + suffix;
        }

        std::string getFileOutPath(std::string name, std::string suffix)
        {
            return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + name + "-" + getPrecAbrev() + "-" + "Out" + suffix;
        }
        std::string getFileValidationPath(std::string name, std::string suffix)
        {
            return ApproxMVBB_TESTS_VALIDATION_FILES_DIR "/" + getPrecAbrev() + "/" + name + "-" + getPrecAbrev() + "-" +
                   "Out" + suffix;
        }

        void dumpOOBB(std::string filePath, const OOBB& oobb)
        {
            std::ofstream l;
            l << std::setprecision(12);
            l.open(filePath.c_str());
            if(!l.good())
            {
                ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
            }

            l << oobb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            l << oobb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            l << oobb.m_q_KI.matrix().format(MyMatrixIOFormat::SpaceSep);

            // Export all corner points in I system
            auto points = oobb.getCornerPoints();
            for(auto& p : points)
            {
                l << std::endl
                  << p.transpose().format(MyMatrixIOFormat::SpaceSep);
            }
            l.close();
        }

        void readOOBB(std::string filePath, Vector3& minP, Vector3& maxP, Matrix33& R_KI, Vector3List& pointList)
        {
            auto v = getPointsFromFile3D(filePath);
            if(v.size() != 5 + 8)
            {
                ApproxMVBB_ERRORMSG("Wrong number of points in OOBB file: " << filePath << " points: " << v.size())
            }
            minP        = v[0];
            maxP        = v[1];
            R_KI.row(0) = v[2];
            R_KI.row(1) = v[3];
            R_KI.row(2) = v[4];

            pointList.resize(8);
            for(unsigned int i = 0; i < 8; ++i)
            {
                pointList[i] = v[5 + i];
            }
        }

        PointFunctions::Vector3List getPointsFromFile3D(std::string filePath)
        {
            std::ifstream file;           // creates stream myFile
            file.open(filePath.c_str());  // opens .txt file

            if(!file.is_open())
            {  // check file is open, quit if not
                ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
            }

            PREC a, b, c;
            Vector3List v;
            while(file.good())
            {
                file >> a >> b >> c;
                v.emplace_back(a, b, c);
            }
            file.close();
            return v;
        }

        Vector2List getPointsFromFile2D(std::string filePath)
        {
            std::ifstream file;           // creates stream myFile
            file.open(filePath.c_str());  // opens .txt file

            if(!file.is_open())
            {  // check file is open, quit if not
                ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
            }
            PREC a, b;
            Vector2List v;
            while(file.good())
            {
                file >> a >> b;
                v.emplace_back(a, b);
            }
            file.close();
            return v;
        }
    }  // namespace TestFunctions
}  // namespace ApproxMVBB
