//////////////////////////////////////////////////////////////////////
// Uses the ASSIMP library to read mesh models in of 30+ file types
// into a structure suitable for the raytracer.
////////////////////////////////////////////////////////////////////////

#include <string>
#include <vector>
#include "raytrace.h"
#include "realtime.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

void recurseModelNodes(Scene* scene,
                       const  aiScene* aiscene,
                       const  aiNode* node,
                       const aiMatrix4x4& parentTr,
                       const int level=0);

void Scene::ReadAssimpFile(const std::vector<double>& f, const std::string path)
{
    // The model being read in can be in *any* coordinate system.  The
    // input parameters specify a model transformation to be applied
    // to the model.  This modelTR is the produce of a (uniform)
    // scale, a rotate, and a translate.
    aiMatrix4x4 T, S;
    aiMatrix4x4::Translation(aiVector3D(float(f[5]), float(f[6]), float(f[7])), T);
    aiMatrix4x4::Scaling(aiVector3D(float(f[4]), float(f[4]), float(f[4])), S);
    aiMatrix4x4 R = aiMatrix4x4(aiQuaternion(float(f[0]), float(f[1]), float(f[2]), float(f[3])).GetMatrix());
    aiMatrix4x4 modelTr = T*R*S;
    
    printf("Reading %s\n", path.c_str()); 
    Assimp::Importer importer;
    const aiScene* aiscene = importer.ReadFile(path.c_str(), aiProcessPreset_TargetRealtime_MaxQuality);
    recurseModelNodes(this, aiscene, aiscene->mRootNode, modelTr);
}

// Recursively traverses the assimp node hierarchy, accumulating
// modeling transformations, and creating and transforming any meshes
// found.  Meshes comming from assimp can have associated surface
// properties, so each mesh *copies* the current BRDF as a starting
// point and modifies it from the assimp data structure.
void recurseModelNodes(Scene* scene,
                       const aiScene* aiscene,
                       const aiNode* node,
                       const aiMatrix4x4& parentTr,
                       const int level)
{
    // Print line with indentation to show structure of the model node hierarchy.
    for (int i=0;  i<level;  i++) printf("| ");
    printf("%s ", node->mName.data);

    // Accumulating transformations while traversing down the hierarchy.
    aiMatrix4x4 childTr = parentTr*node->mTransformation;
    aiMatrix3x3 normalTr = aiMatrix3x3(childTr); // Really should be inverse-transpose for full generality
     
    // Loop through this node's meshes
    for (unsigned int i=0;  i<node->mNumMeshes; ++i) {
        aiMesh* aimesh = aiscene->mMeshes[node->mMeshes[i]];
        printf("%d:%d ", aimesh->mNumVertices, aimesh->mNumFaces);

        // Pass this node's surface material into the scene.
        scene->createMaterial();
        aiString texPath;
        aiMaterial* mtl = aiscene->mMaterials[aimesh->mMaterialIndex];
        // aiColor4D c;  
        // float s;
        // if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &c))
        //     scene-setKd(glm::vec3(c.r, c.g, c.b));
        // if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &c))
        //     scene->setKs(glm::vec3(c.r, c.g, c.b));
        // if (AI_SUCCESS == mtl->Get(AI_MATKEY_SHININESS, &s, NULL))
        //     scene->setAlpha(s);
        if (AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, 0, &texPath))
            scene->setTexture(texPath.C_Str());  

        // Arrays to hold all vertex and triangle data.  
        std::vector<float>* pnt = new std::vector<float>;
        std::vector<float>* nrm = new std::vector<float>;
        std::vector<float>* tex = new std::vector<float>;
        std::vector<float>* tan = new std::vector<float>;
        std::vector<unsigned int>* tris = new std::vector<unsigned int>;
        
        // Loop through all vertices and record the
        // vertex/normal/texture/tangent data with the node's model
        // transformation applied.
        for (unsigned int t=0;  t<aimesh->mNumVertices;  ++t) {
            aiVector3D aipnt = childTr*aimesh->mVertices[t];
            aiVector3D ainrm = normalTr*aimesh->mNormals[t];
            aiVector3D aitex = aimesh->HasTextureCoords(0) ? aimesh->mTextureCoords[0][t] : aiVector3D(0,0,0);
            aiVector3D aitan = normalTr*aiVector3D(1,0,0); // BOGUS!  Does ASSIMP supply this? (Yes, I think.)

            pnt->push_back(aipnt.x);
            pnt->push_back(aipnt.y);
            pnt->push_back(aipnt.z);
            pnt->push_back(1.0f);

            nrm->push_back(ainrm.x);
            nrm->push_back(ainrm.y);
            nrm->push_back(ainrm.z);

            tex->push_back(aitex.x);
            tex->push_back(aitex.y);

            tan->push_back(aitan.x);
            tan->push_back(aitan.y);
            tan->push_back(aitan.z); }
        
        // Loop through all faces, recording indices
        for (unsigned int t=0;  t<aimesh->mNumFaces;  ++t) {
            aiFace* aiface = &aimesh->mFaces[t];
            tris->push_back(aiface->mIndices[0]);
            tris->push_back(aiface->mIndices[1]);
            tris->push_back(aiface->mIndices[2]); }
        
        scene->triangleMesh(pnt, nrm, tex, tan, tris); }

    printf("\n");

    // Recurse onto this node's children
    for (unsigned int i=0;  i<node->mNumChildren;  ++i)
      recurseModelNodes(scene, aiscene, node->mChildren[i], childTr, level+1);
}
