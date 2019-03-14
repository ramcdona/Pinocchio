/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SKELETON_H_BFCF2002_4190_11E9_AA8F_EFB66606E782
#define SKELETON_H_BFCF2002_4190_11E9_AA8F_EFB66606E782

#include <map>
#include "graphutils.h"

namespace Pinocchio {

class PINOCCHIO_API Skeleton
{
  public:
    Skeleton() {}

    const PtGraph &fGraph() const { return fGraphV; }
    const std::vector<int> &fPrev() const { return fPrevV; }
    const std::vector<int> &fSym() const { return fSymV; }

    const PtGraph &cGraph() const { return cGraphV; }
    const std::vector<int> &cPrev() const { return cPrevV; }
    const std::vector<int> &cSym() const { return cSymV; }
    const std::vector<bool> &cFeet() const { return cFeetV; }
    const std::vector<bool> &cFat() const { return cFatV; }

    const std::vector<int> &cfMap() const { return cfMapV; }
    const std::vector<int> &fcMap() const { return fcMapV; }
    const std::vector<double> &fcFraction() const { return fcFractionV; }
    const std::vector<double> &cLength() const { return cLengthV; }

    int getJointForName(const std::string &name) const { if(jointNames.count(name)) return jointNames.find(name)->second; return -1; }

    void scale(double factor);

  protected:
    void initCompressed();

    //help for creation
    std::map<std::string, int> jointNames;
    void makeJoint(const std::string &name, const Vector3 &pos, const std::string &previous = std::string());
    void makeSymmetric(const std::string &name1, const std::string &name2);
    void setFoot(const std::string &name);
    void setFat(const std::string &name);

  private:
    //full
    PtGraph fGraphV;
    //previous vertices
    std::vector<int> fPrevV;
    //symmetry
    std::vector<int> fSymV;

    //compressed (no degree 2 vertices)
    PtGraph cGraphV;
    //previous vertices
    std::vector<int> cPrevV;
    //symmetry
    std::vector<int> cSymV;
    //whether the vertex should be near the ground
    std::vector<bool> cFeetV;
    //whether the vertex should be in a large region
    std::vector<bool> cFatV;

    //compressed to full std::map
    std::vector<int> cfMapV;
    //full to compressed std::map, -1 when vertex is not in compressed
    std::vector<int> fcMapV;
    //maps full vertex number to ratio of its prev edge length to total length of
    std::vector<double> fcFractionV;
    //containing edge in the compressed graph
    //lengths of the compressed bones
    std::vector<double> cLengthV;
};

class PINOCCHIO_API HumanSkeleton : public Skeleton
{
  public:
    HumanSkeleton();
};

class PINOCCHIO_API QuadSkeleton : public Skeleton
{
  public:
    QuadSkeleton();
};

class PINOCCHIO_API HorseSkeleton : public Skeleton
{
  public:
    HorseSkeleton();
};

class PINOCCHIO_API CentaurSkeleton : public Skeleton
{
  public:
    CentaurSkeleton();
};

class PINOCCHIO_API FileSkeleton : public Skeleton
{
  public:
    FileSkeleton(const std::string &filename);
};

} // namespace Pinocchio

#endif // SKELETON_H_BFCF2002_4190_11E9_AA8F_EFB66606E782
