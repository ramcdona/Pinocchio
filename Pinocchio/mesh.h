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

#ifndef MESH_H_BFCF2002_4190_11E9_AA8F_EFB66606E782
#define MESH_H_BFCF2002_4190_11E9_AA8F_EFB66606E782

#include <vector>

#include "vector.h"
#include "rect.h"

//#define

struct MeshVertex
{
  MeshVertex() : edge(-1) {}

  Vector3 pos;
  Vector3 normal;
  //an edge such that edge->prev->vertex is this
  int edge;
};

struct MeshEdge
{
  MeshEdge() : vertex(-1), prev(-1), twin(-1) {}

  //the vertex the edge points to--the start vertex is prev->vertex
  int vertex;
  //ccw, next is prev->prev
  int prev;
  int twin;
};

class PINOCCHIO_API Mesh
{
  public:
    Mesh() : scale(1.) {}
    Mesh(const std::string &file, int algo=Mesh::LBS, float weight=1.);

    bool integrityCheck() const;
    //returns true if the mesh consists of a single connected component
    bool isConnected() const;
    void computeVertexNormals();
    void normalizeBoundingBox();
    void computeTopology();
    void writeObj(const std::string &filename) const;

  private:
    void readObj(std::istream &strm);
    void readOff(std::istream &strm);
    void readPly(std::istream &strm);
    void readGts(std::istream &strm);
    void readStl(std::istream &strm);
    void fixDupFaces();
    //sort edges so that triplets forming faces are adjacent
    void sortEdges();

    //data
  public:
    std::vector<MeshVertex> vertices;
    //halfEdges, really
    std::vector<MeshEdge> edges;

    Vector3 toAdd;
    double scale;
    float blendWeight;
    int algo;
    static int LBS;
    static int DQS;
    static int MIX;
};
#endif
