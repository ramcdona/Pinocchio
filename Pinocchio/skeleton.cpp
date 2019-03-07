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

/* 06.11.2014 RR: * makeJoint(), if defineed PINOCCHIO_SKEL_XYZ_ROT
                    rotate the positon componentes for the joints.
                    X Y Z  -->  Z Y X
*/

#include "skeleton.h"
#include "utils.h"
#include "debugging.h"
#include <fstream>

void Skeleton::initCompressed()
{
  int i;

  fcMapV.resize(fPrevV.size(), -1);
  fcFractionV.resize(fPrevV.size(), -1.);

  for(i = 0; i < (int)fPrevV.size(); ++i)
  {
    if(fGraphV.edges[i].size() == 2)
      continue;
    fcMapV[i] = cfMapV.size();
    cfMapV.push_back(i);
  }

  cPrevV.resize(cfMapV.size(), -1);
  cSymV.resize(cfMapV.size(), -1);
  cGraphV.edges.resize(cfMapV.size());
  cFeetV = std::vector<bool>(cPrevV.size(), false);
  cFatV = std::vector<bool>(cPrevV.size(), false);

  for(i = 0; i < (int)cfMapV.size(); ++i)
  {
    cGraphV.verts.push_back(fGraphV.verts[cfMapV[i]]);

    //symmetry--TODO: need to make sure all unreduced bones in chain
    //          are marked as symmetric before marking the reduced one
    if(fSymV[cfMapV[i]] >= 0)
      cSymV[i] = fcMapV[fSymV[cfMapV[i]]];

    //prev
    if(i > 0)
    {
      int curPrev = fPrevV[cfMapV[i]];
      while(fcMapV[curPrev]  < 0)
        curPrev = fPrevV[curPrev];
      cPrevV[i] = fcMapV[curPrev];
    }
  }

  //graph edges
  for(i = 1; i < (int)cPrevV.size(); ++i)
  {
    cGraphV.edges[i].push_back(cPrevV[i]);
    cGraphV.edges[cPrevV[i]].push_back(i);
  }

  cLengthV.resize(cPrevV.size(), 0.);

  //lengths/fraction computation
  for(i = 1; i < (int)cPrevV.size(); ++i)
  {
    int cur = cfMapV[i];
    _HASH_NAMESPACE::hash_map<int, double> lengths;
    do
    {
      lengths[cur] = (fGraphV.verts[cur] - fGraphV.verts[fPrevV[cur]]).length();
      cLengthV[i] += lengths[cur];
      cur = fPrevV[cur];
    } while(fcMapV[cur] == -1);

    for(_HASH_NAMESPACE::hash_map<int, double>::iterator it = lengths.begin(); it != lengths.end(); ++it)
      fcFractionV[it->first] = it->second / cLengthV[i];
  }
}


void Skeleton::scale(double factor)
{
  int i;
  for(i = 0; i < (int)fGraphV.verts.size(); ++i)
    fGraphV.verts[i] *= factor;
  for(i = 0; i < (int)cGraphV.verts.size(); ++i)
  {
    cGraphV.verts[i] *= factor;
    cLengthV[i] *= factor;
  }
}


void Skeleton::makeJoint(const std::string &name, const Vector3 &pos, const std::string &previous)
{
  int cur = fSymV.size();
  fSymV.push_back(-1);

  #if PINOCCHIO_SKEL_XYZ_ROT
  //skeletons specified in [-1,1] will be fit to object in [0,1]
  fGraphV.verts.push_back(Vector3( pos[2], pos[0], pos[1]) * 0.5);
  #else
  //skeletons specified in [-1,1] will be fit to object in [0,1]
  fGraphV.verts.push_back(pos * 0.5);
  #endif

  fGraphV.edges.resize(cur + 1);
  jointNames[name] = cur;

  if(previous == std::string(""))
  {
    fPrevV.push_back(-1);
    //add a bone
  }
  else
  {
    int prev = jointNames[previous];
    fGraphV.edges[cur].push_back(prev);
    fGraphV.edges[prev].push_back(cur);
    fPrevV.push_back(prev);
  }
}


void Skeleton::makeSymmetric(const std::string &name1, const std::string &name2)
{
  int i1 = jointNames[name1];
  int i2 = jointNames[name2];

  if(i1 > i2)
    std::swap(i1, i2);
  fSymV[i2] = i1;
}


void Skeleton::setFoot(const std::string &name)
{
  int i = jointNames[name];
  cFeetV[fcMapV[i]] = true;
}


void Skeleton::setFat(const std::string &name)
{
  int i = jointNames[name];
  cFatV[fcMapV[i]] = true;
}


//-----------------actual skeletons-------------------

HumanSkeleton::HumanSkeleton()
{
  //order of makeJoint calls is very important
  makeJoint("shoulders",  Vector3(0., 0.5, 0.));
  makeJoint("back",       Vector3(0., 0.15, 0.),      "shoulders");
  makeJoint("hips",       Vector3(0., 0., 0.),        "back");
  makeJoint("head",       Vector3(0., 0.7, 0.),       "shoulders");

  makeJoint("lthigh",     Vector3(-0.1, 0., 0.),      "hips");
  makeJoint("lknee",      Vector3(-0.15, -0.35, 0.),  "lthigh");
  makeJoint("lankle",      Vector3(-0.15, -0.8, 0.),  "lknee");
  makeJoint("lfoot",      Vector3(-0.15, -0.8, 0.1),  "lankle");

  makeJoint("rthigh",     Vector3(0.1, 0., 0.),       "hips");
  makeJoint("rknee",      Vector3(0.15, -0.35, 0.),   "rthigh");
  makeJoint("rankle",      Vector3(0.15, -0.8, 0.),   "rknee");
  makeJoint("rfoot",      Vector3(0.15, -0.8, 0.1),   "rankle");

  makeJoint("lshoulder",  Vector3(-0.2, 0.5, 0.),     "shoulders");
  makeJoint("lelbow",     Vector3(-0.4, 0.25, 0.075), "lshoulder");
  makeJoint("lhand",      Vector3(-0.6, 0.0, 0.15),   "lelbow");

  makeJoint("rshoulder",  Vector3(0.2, 0.5, 0.),      "shoulders");
  makeJoint("relbow",     Vector3(0.4, 0.25, 0.075),  "rshoulder");
  makeJoint("rhand",      Vector3(0.6, 0.0, 0.15),    "relbow");

  //symmetry
  makeSymmetric("lthigh", "rthigh");
  makeSymmetric("lknee", "rknee");
  makeSymmetric("lankle", "rankle");
  makeSymmetric("lfoot", "rfoot");

  makeSymmetric("lshoulder", "rshoulder");
  makeSymmetric("lelbow", "relbow");
  makeSymmetric("lhand", "rhand");

  initCompressed();

  setFoot("lfoot");
  setFoot("rfoot");

  setFat("hips");
  setFat("shoulders");
  setFat("head");
}


QuadSkeleton::QuadSkeleton()
{

  #ifndef USE_KINECSDK_ORDER

  //order of makeJoint calls is very important
  makeJoint("shoulders",  Vector3(0., 0.5, 0.));
  makeJoint("back",       Vector3(0., 0.15, 0.),      "shoulders");
  makeJoint("hips",       Vector3(0., 0., 0.),        "back");
  makeJoint("head",       Vector3(0., 0.7, 0.),       "shoulders");

  makeJoint("lthigh",     Vector3(-0.1, 0., 0.),      "hips");
  makeJoint("lknee",      Vector3(-0.15, -0.35, 0.),  "lthigh");
  makeJoint("lankle",      Vector3(-0.15, -0.8, 0.),  "lknee");
  makeJoint("lfoot",      Vector3(-0.15, -0.8, 0.1),  "lankle");

  makeJoint("rthigh",     Vector3(0.1, 0., 0.),       "hips");
  makeJoint("rknee",      Vector3(0.15, -0.35, 0.),   "rthigh");
  makeJoint("rankle",      Vector3(0.15, -0.8, 0.),   "rknee");
  makeJoint("rfoot",      Vector3(0.15, -0.8, 0.1),   "rankle");

  makeJoint("lshoulder",  Vector3(-0.2, 0.5, 0.),     "shoulders");
  makeJoint("lelbow",     Vector3(-0.4, 0.25, 0.075), "lshoulder");
  makeJoint("lhand",      Vector3(-0.6, 0.0, 0.15),   "lelbow");

  makeJoint("rshoulder",  Vector3(0.2, 0.5, 0.),      "shoulders");
  makeJoint("relbow",     Vector3(0.4, 0.25, 0.075),  "rshoulder");
  makeJoint("rhand",      Vector3(0.6, 0.0, 0.15),    "relbow");

  //symmetry
  makeSymmetric("lthigh", "rthigh");
  makeSymmetric("lknee", "rknee");
  makeSymmetric("lankle", "rankle");
  makeSymmetric("lfoot", "rfoot");

  makeSymmetric("lshoulder", "rshoulder");
  makeSymmetric("lelbow", "relbow");
  makeSymmetric("lhand", "rhand");

  #else

  //order of makeJoint calls is very important
  makeJoint("shoulders",  Vector3(0., 0., 0.5));
  makeJoint("back",       Vector3(0., 0., 0.),         "shoulders");
  makeJoint("hips",       Vector3(0., 0., -0.5),       "back");
  makeJoint("neck",       Vector3(0., 0.2, 0.63),      "shoulders");
  makeJoint("head",       Vector3(0., 0.2, 0.9),       "neck");

  makeJoint("lthigh",     Vector3(-0.15, 0., -0.5),     "hips");
  makeJoint("lhknee",     Vector3(-0.2, -0.4, -0.5),   "lthigh");
  makeJoint("lhfoot",     Vector3(-0.2, -0.8, -0.5),   "lhknee");

  makeJoint("rthigh",     Vector3(0.15, 0., -0.5),      "hips");
  makeJoint("rhknee",     Vector3(0.2, -0.4, -0.5),    "rthigh");
  makeJoint("rhfoot",     Vector3(0.2, -0.8, -0.5),    "rhknee");

  makeJoint("lshoulder",  Vector3(-0.2, 0., 0.5),      "shoulders");
  makeJoint("lfknee",     Vector3(-0.2, -0.4, 0.5),    "lshoulder");
  makeJoint("lffoot",      Vector3(-0.2, -0.8, 0.5),   "lfknee");

  makeJoint("rshoulder",  Vector3(0.2, 0.0, 0.5),      "shoulders");
  makeJoint("rfknee",     Vector3(0.2, -0.4, 0.5),     "rshoulder");
  makeJoint("rffoot",      Vector3(0.2, -0.8, 0.5),    "rfknee");

  makeJoint("tail",       Vector3(0., 0., -0.7),       "hips");

  //symmetry
  makeSymmetric("lthigh", "rthigh");
  makeSymmetric("lhknee", "rhknee");
  makeSymmetric("lhfoot", "rhfoot");

  makeSymmetric("lshoulder", "rshoulder");
  makeSymmetric("lfknee", "rfknee");
  makeSymmetric("lffoot", "rffoot");

  #endif

  initCompressed();

  setFoot("lhfoot");
  setFoot("rhfoot");
  setFoot("lffoot");
  setFoot("rffoot");

  setFat("hips");
  setFat("shoulders");
  setFat("head");
}


HorseSkeleton::HorseSkeleton()
{
  //order of makeJoint calls is very important
  makeJoint("shoulders",  Vector3(0., 0., 0.5));
  makeJoint("back",       Vector3(0., 0., 0.),         "shoulders");
  makeJoint("hips",       Vector3(0., 0., -0.5),       "back");
  makeJoint("neck",       Vector3(0., 0.2, 0.63),      "shoulders");
  makeJoint("head",       Vector3(0., 0.2, 0.9),       "neck");

  makeJoint("lthigh",     Vector3(-0.15, 0., -0.5),     "hips");
  makeJoint("lhknee",     Vector3(-0.2, -0.2, -0.45),  "lthigh");
  makeJoint("lhheel",     Vector3(-0.2, -0.4, -0.5),   "lhknee");
  makeJoint("lhfoot",     Vector3(-0.2, -0.8, -0.5),   "lhheel");

  makeJoint("rthigh",     Vector3(0.15, 0., -0.5),      "hips");
  makeJoint("rhknee",     Vector3(0.2, -0.2, -0.45),   "rthigh");
  makeJoint("rhheel",     Vector3(0.2, -0.4, -0.5),    "rhknee");
  makeJoint("rhfoot",     Vector3(0.2, -0.8, -0.5),    "rhheel");

  makeJoint("lshoulder",  Vector3(-0.2, 0., 0.5),      "shoulders");
  makeJoint("lfknee",     Vector3(-0.2, -0.4, 0.5),    "lshoulder");
  makeJoint("lffoot",      Vector3(-0.2, -0.8, 0.5),   "lfknee");

  makeJoint("rshoulder",  Vector3(0.2, 0.0, 0.5),      "shoulders");
  makeJoint("rfknee",     Vector3(0.2, -0.4, 0.5),     "rshoulder");
  makeJoint("rffoot",      Vector3(0.2, -0.8, 0.5),    "rfknee");

  makeJoint("tail",       Vector3(0., 0., -0.7),       "hips");

  //symmetry
  makeSymmetric("lthigh", "rthigh");
  makeSymmetric("lhknee", "rhknee");
  makeSymmetric("lhheel", "rhheel");
  makeSymmetric("lhfoot", "rhfoot");

  makeSymmetric("lshoulder", "rshoulder");
  makeSymmetric("lfknee", "rfknee");
  makeSymmetric("lffoot", "rffoot");

  initCompressed();

  setFoot("lhfoot");
  setFoot("rhfoot");
  setFoot("lffoot");
  setFoot("rffoot");

  setFat("hips");
  setFat("shoulders");
  setFat("head");
}


CentaurSkeleton::CentaurSkeleton()
{
  //order of makeJoint calls is very important
  //0
  makeJoint("shoulders",  Vector3(0., 0., 0.5));
  //1
  makeJoint("back",       Vector3(0., 0., 0.),         "shoulders");
  //2
  makeJoint("hips",       Vector3(0., 0., -0.5),       "back");

  //3
  makeJoint("hback",      Vector3(0., 0.25, 0.5),      "shoulders");
  //4
  makeJoint("hshoulders", Vector3(0., 0.5, 0.5),       "hback");
  //5
  makeJoint("head",       Vector3(0., 0.7, 0.5),       "hshoulders");

  //6
  makeJoint("lthigh",     Vector3(-0.15, 0., -0.5),    "hips");
  //7
  makeJoint("lhknee",     Vector3(-0.2, -0.4, -0.45),  "lthigh");
  //8
  makeJoint("lhfoot",     Vector3(-0.2, -0.8, -0.5),   "lhknee");

  //9
  makeJoint("rthigh",     Vector3(0.15, 0., -0.5),     "hips");
  //10
  makeJoint("rhknee",     Vector3(0.2, -0.4, -0.45),   "rthigh");
  //11
  makeJoint("rhfoot",     Vector3(0.2, -0.8, -0.5),    "rhknee");

  //12
  makeJoint("lshoulder",  Vector3(-0.2, 0., 0.5),      "shoulders");
  //13
  makeJoint("lfknee",     Vector3(-0.2, -0.4, 0.5),    "lshoulder");
  //14
  makeJoint("lffoot",     Vector3(-0.2, -0.8, 0.5),    "lfknee");

  //15
  makeJoint("rshoulder",  Vector3(0.2, 0.0, 0.5),      "shoulders");
  //16
  makeJoint("rfknee",     Vector3(0.2, -0.4, 0.5),     "rshoulder");
  //17
  makeJoint("rffoot",     Vector3(0.2, -0.8, 0.5),     "rfknee");

  //18
  makeJoint("hlshoulder", Vector3(-0.2, 0.5, 0.5),     "hshoulders");
  //19
  makeJoint("lelbow",     Vector3(-0.4, 0.25, 0.575),  "hlshoulder");
  //20
  makeJoint("lhand",      Vector3(-0.6, 0.0, 0.65),    "lelbow");

  //21
  makeJoint("hrshoulder", Vector3(0.2, 0.5, 0.5),      "hshoulders");
  //22
  makeJoint("relbow",     Vector3(0.4, 0.25, 0.575),   "hrshoulder");
  //23
  makeJoint("rhand",      Vector3(0.6, 0.0, 0.65),     "relbow");

  //24
  makeJoint("tail",       Vector3(0., 0., -0.7),       "hips");

  //symmetry
  makeSymmetric("lthigh", "rthigh");
  makeSymmetric("lhknee", "rhknee");
  makeSymmetric("lhheel", "rhheel");
  makeSymmetric("lhfoot", "rhfoot");

  makeSymmetric("lshoulder", "rshoulder");
  makeSymmetric("lfknee", "rfknee");
  makeSymmetric("lffoot", "rffoot");

  makeSymmetric("hlshoulder", "hrshoulder");
  makeSymmetric("lelbow", "relbow");
  makeSymmetric("lhand", "rhand");

  initCompressed();

  setFoot("lhfoot");
  setFoot("rhfoot");
  setFoot("lffoot");
  setFoot("rffoot");

  setFat("hips");
  setFat("shoulders");
  setFat("hshoulders");
  setFat("head");
}


FileSkeleton::FileSkeleton(const std::string &filename)
{
  std::ifstream strm(filename.c_str());

  if(!strm.is_open())
  {
    Debugging::out() << "Error opening file " << filename << std::endl;
    return;
  }

  while(!strm.eof())
  {
    std::vector<std::string> line = readWords(strm);
    if(line.size() < 5)
      //error
      continue;

    Vector3 p;
    sscanf(line[1].c_str(), "%lf", &(p[0]));
    sscanf(line[2].c_str(), "%lf", &(p[1]));
    sscanf(line[3].c_str(), "%lf", &(p[2]));

    if(line[4] == "-1")
      line[4] = std::string();

    makeJoint(line[0], p * 2., line[4]);
  }

  initCompressed();
}
