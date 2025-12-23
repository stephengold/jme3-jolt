/*
 Copyright (c) 2025 Stephen Gold

 BSD 3-Clause License

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.github.stephengold.jme3jolt;

import com.github.stephengold.joltjni.BoxShapeSettings;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.HeightFieldShapeSettings;
import com.github.stephengold.joltjni.IndexedTriangle;
import com.github.stephengold.joltjni.IndexedTriangleList;
import com.github.stephengold.joltjni.MeshShapeSettings;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.ScaledShapeSettings;
import com.github.stephengold.joltjni.ShapeSettings;
import com.github.stephengold.joltjni.SphereShapeSettings;
import com.github.stephengold.joltjni.StaticCompoundShapeSettings;
import com.github.stephengold.joltjni.TriangleShapeSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.VertexList;
import com.github.stephengold.joltjni.vhacd.ConvexHull;
import com.github.stephengold.joltjni.vhacd.Decomposer;
import com.github.stephengold.joltjni.vhacd.Parameters;
import com.jme3.bounding.BoundingBox;
import com.jme3.bounding.BoundingSphere;
import com.jme3.bounding.BoundingVolume;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.UserData;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.terrain.Terrain;
import com.jme3.util.BufferUtils;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;

/**
 * Utility methods to build shape settings for physics objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class ShapeBuilder {
    // *************************************************************************
    // constants

    /**
     * number of components in a 3-D vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ShapeBuilder() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Build shape settings for a static body.
     *
     * @param modelRoot the model to match (not null, unaffected)
     * @return a new settings object
     */
    public static ShapeSettings forStaticBody(Spatial modelRoot) {
        ShapeSettings result;
        if (modelRoot instanceof Terrain) { // must precede test for Geometry
            result = fromTerrain((Terrain) modelRoot);
            return result;

        } else if (modelRoot instanceof Geometry) {
            result = fromGeometry((Geometry) modelRoot);

        } else if (modelRoot instanceof Node) {
            result = fromNode((Node) modelRoot);

        } else {
            throw new IllegalArgumentException(
                    "The model root must either be a Node or a Geometry!");
        }

        return result;
    }

    /**
     * Build shape settings to match the bounding volume of the specified model.
     *
     * @param modelRoot the model to match (not null, unaffected)
     * @return a new settings object
     */
    public static ShapeSettings fromBoundingVolume(Spatial modelRoot) {
        BoundingVolume volume = modelRoot.getWorldBound();

        ShapeSettings result;
        if (volume instanceof BoundingBox) {
            BoundingBox box = (BoundingBox) volume;
            Vector3f halfExtents = box.getExtent(null);
            result = new BoxShapeSettings(
                    halfExtents.x, halfExtents.y, halfExtents.z);
        } else if (volume instanceof BoundingSphere) {
            BoundingSphere sphere = (BoundingSphere) volume;
            float radius = sphere.getRadius();
            result = new SphereShapeSettings(radius);
        } else {
            throw new IllegalStateException(volume.getClass().getSimpleName());
        }

        return result;
    }

    /**
     * Build settings for a moving body, to match the convex hull of the
     * specified model.
     *
     * @param modelRoot the model to match (not null, unaffected)
     * @return a new settings object
     */
    public static ShapeSettings fromConvexHull(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        int numPoints = mergedMesh.getVertexCount();
        FloatBuffer floats
                = mergedMesh.getFloatBuffer(VertexBuffer.Type.Position);
        ShapeSettings result = new ConvexHullShapeSettings(numPoints, floats);

        return result;
    }

    /**
     * Build shape settings for a moving body, to match the triangles of the
     * specified model. This is an experimental feature, probably not a good
     * idea for most applications.
     *
     * @param modelRoot the model to match (not null, unaffected)
     * @return a new settings object
     */
    public static ShapeSettings fromTriangles(Spatial modelRoot) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);
        assert mergedMesh.getMode() == Mesh.Mode.Triangles :
                mergedMesh.getMode();
        IndexBuffer indexBuffer = mergedMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        int numTriangles = numIndices / vpt;
        assert numTriangles * vpt == numIndices : numIndices;
        Vector3f location = Temps.vector3f.get();

        StaticCompoundShapeSettings result = new StaticCompoundShapeSettings();
        for (int triIndex = 0; triIndex < numTriangles; ++triIndex) {
            int triOffset = vpt * triIndex;
            TriangleShapeSettings triangle = new TriangleShapeSettings();

            int vi0 = indexBuffer.get(triOffset);
            MyMesh.vertexVector3f(
                    mergedMesh, VertexBuffer.Type.Position, vi0, location);
            triangle.setV1(location.x, location.y, location.z);

            int vi1 = indexBuffer.get(triOffset + 1);
            MyMesh.vertexVector3f(
                    mergedMesh, VertexBuffer.Type.Position, vi1, location);
            triangle.setV2(location.x, location.y, location.z);

            int vi2 = indexBuffer.get(triOffset + 2);
            MyMesh.vertexVector3f(
                    mergedMesh, VertexBuffer.Type.Position, vi2, location);
            triangle.setV3(location.x, location.y, location.z);

            result.addShape(0f, 0f, 0f, triangle);
        }

        return result;
    }

    /**
     * Build shape settings for a moving body, using V-HACD. This is potentially
     * very time consuming.
     *
     * @param modelRoot the model to match (not null, unaffected)
     * @param parameters the V-HACD parameters to use (not null, unaffected)
     * @return a new settings object
     */
    public static ShapeSettings usingVhacd(
            Spatial modelRoot, Parameters parameters) {
        Validate.nonNull(modelRoot, "model root");

        Mesh mergedMesh = makeMergedMesh(modelRoot);

        FloatBuffer positionBuffer
                = mergedMesh.getFloatBuffer(VertexBuffer.Type.Position);
        int numFloats = positionBuffer.limit();
        float[] positionArray = new float[numFloats];
        for (int offset = 0; offset < numFloats; ++offset) {
            positionArray[offset] = positionBuffer.get(offset);
        }

        IndexBuffer indexBuffer = mergedMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        int[] indexArray = new int[numIndices];
        for (int offset = 0; offset < numIndices; ++offset) {
            indexArray[offset] = indexBuffer.get(offset);
        }

        // Use the V-HACD algorithm to generate a collection of hulls:
        Decomposer decomposer = new Decomposer();
        Collection<ConvexHull> vhacdHulls
                = decomposer.decompose(positionArray, indexArray, parameters);
        /*
         * Convert each V-HACD hull to a ConvexHullShapeSettings
         * and add that to the result:
         */
        StaticCompoundShapeSettings result = new StaticCompoundShapeSettings();
        result.addHulls(vhacdHulls);

        return result;
    }
    // *************************************************************************
    // private methods - TODO move most of these to the Heart library

    /**
     * Append transformed mesh triangles to a merged mesh.
     *
     * @param geometry the Geometry from which to read triangles (not null,
     * unaffected)
     * @param modelRoot the scene-graph subtree from which to extract triangles
     * (not null, unaffected)
     * @param addPositions the position buffer for the merged mesh (not null,
     * modified)
     * @param addIndices the index buffer for the merged mesh (not null,
     * modified)
     */
    private static void appendTriangles(Geometry geometry, Spatial modelRoot,
            FloatBuffer addPositions, IndexBuffer addIndices) {
        Mesh jmeMesh = geometry.getMesh();
        if (!MyMesh.hasTriangles(jmeMesh)) {
            return;
        }

        // Append merged-mesh indices to the IndexBuffer:
        int indexBase = addPositions.position() / numAxes;
        IndexBuffer indexBuffer = jmeMesh.getIndicesAsList();
        int numIndices = indexBuffer.size();
        for (int offset = 0; offset < numIndices; ++offset) {
            int indexInGeometry = indexBuffer.get(offset);
            int indexInMergedMesh = indexBase + indexInGeometry;
            addIndices.put(indexInMergedMesh);
        }

        // Append transformed vertex locations to the FloatBuffer:
        Transform transform = relativeTransform(geometry, modelRoot);
        Vector3f tmpPosition = new Vector3f();
        int numVertices = jmeMesh.getVertexCount();
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            MyMesh.vertexVector3f(jmeMesh, VertexBuffer.Type.Position,
                    vertexIndex, tmpPosition);
            MyMath.transform(transform, tmpPosition, tmpPosition);
            addPositions.put(tmpPosition.x);
            addPositions.put(tmpPosition.y);
            addPositions.put(tmpPosition.z);
        }
    }

    /**
     * Generate mesh-shape settings from the specified Geometry.
     *
     * @param geometry the geometry to use (not null)
     * @return a new settings object
     */
    private static MeshShapeSettings fromGeometry(Geometry geometry) {
        Mesh jmeMesh = geometry.getMesh();
        IndexBuffer indexBuffer = jmeMesh.getIndicesAsList();

        IndexedTriangleList indices = new IndexedTriangleList();
        VertexList vertices = new VertexList();

        int numTriangles;
        if (MyMesh.hasTriangles(jmeMesh)) {
            numTriangles = indexBuffer.size() / vpt;
            assert numTriangles == jmeMesh.getTriangleCount() : numTriangles;
            indices.resize(numTriangles);

            for (int triIndex = 0; triIndex < numTriangles; ++triIndex) {
                IndexedTriangle it = indices.get(triIndex);
                int vi0 = indexBuffer.get(triIndex * vpt);
                it.setIdx(0, vi0);
                int vi1 = indexBuffer.get(triIndex * vpt + 1);
                it.setIdx(1, vi1);
                int vi2 = indexBuffer.get(triIndex * vpt + 2);
                it.setIdx(2, vi2);
            }

            Vector3f vertex = Temps.vector3f.get();
            int numVertices = jmeMesh.getVertexCount();
            vertices.resize(numVertices);
            for (int vi = 0; vi < numVertices; ++vi) {
                MyMesh.vertexVector3f(
                        jmeMesh, VertexBuffer.Type.Position, vi, vertex);
                vertices.set(vi, vertex.x, vertex.y, vertex.z);
            }
        }
        MeshShapeSettings result = new MeshShapeSettings(vertices, indices);

        return result;
    }

    /**
     * Generate compound-shape settings from the specified scene-graph node.
     *
     * @param parent the node to use (not null)
     * @return a new settings object
     */
    private static StaticCompoundShapeSettings fromNode(Node parent) {
        Quat joltRotation = Temps.quat.get();
        Vec3 joltVector = Temps.vec3.get();
        StaticCompoundShapeSettings result = new StaticCompoundShapeSettings();
        for (Spatial child : parent.getChildren()) {
            Boolean skipChild = child.getUserData(UserData.JME_PHYSICSIGNORE);
            if (skipChild != null && skipChild) {
                continue; // to the next child spatial
            }

            ShapeSettings subShape = forStaticBody(child);
            Transform transform = child.getLocalTransform();
            Vector3f scale = transform.getScale(); // alias
            if (!MyVector3f.isScaleIdentity(scale)) {
                joltVector.set(scale.x, scale.y, scale.z);
                subShape = new ScaledShapeSettings(subShape, joltVector);
            }

            Vector3f offset = transform.getTranslation(); // alias
            joltVector.set(offset.x, offset.y, offset.z);
            Quaternion rotation = transform.getRotation(); // alias
            joltRotation.set(rotation.getX(), rotation.getY(), rotation.getZ(),
                    rotation.getW());
            result.addShape(joltVector, joltRotation, subShape);
        }

        return result;
    }

    /**
     * Generate heightfield shape settings from a scene-graph terrain.
     *
     * @param terrain the terrain to use (not null)
     * @return a new settings object
     */
    private static HeightFieldShapeSettings fromTerrain(Terrain terrain) {
        float[] sampleArray = terrain.getHeightMap();
        Vec3 offset = new Vec3();
        Vec3 scale = Vec3.sOne();
        int sampleCount = (int) FastMath.sqrt(sampleArray.length);
        HeightFieldShapeSettings result = new HeightFieldShapeSettings(
                sampleArray, offset, scale, sampleCount);

        return result;
    }

    /**
     * Enumerate all child geometries in the specified scene-graph subtree that
     * aren't tagged with {@code UserData.JME_PHYSICSIGNORE}. Note: recursive!
     *
     * @param parent the parent of the subtree (not null, aliases created)
     * @param addResult storage for results (added to if not null)
     * @return a new List
     */
    private static List<Geometry> listUntaggedGeometries(
            Node parent, List<Geometry> addResult) {
        Validate.nonNull(parent, "subtree");
        List<Geometry> result
                = (addResult == null) ? new ArrayList<Geometry>(50) : addResult;

        for (Spatial child : parent.getChildren()) {
            Boolean skipChild = child.getUserData(UserData.JME_PHYSICSIGNORE);
            if (skipChild != null && skipChild) {
                // continue to the next child spatial

            } else if (child instanceof Node) {
                listUntaggedGeometries((Node) child, result);

            } else {
                result.add((Geometry) child);
            }
        }

        return result;
    }

    /**
     * Generate a mesh that merges the triangles of non-empty geometries that
     * aren't tagged with {@code UserData.JME_PHYSICSIGNORE}.
     *
     * @param modelRoot the scene-graph subtree from which to generate the mesh
     * (not null, unaffected)
     * @return a new, indexed Mesh in Triangles mode, its bounds not set
     */
    private static Mesh makeMergedMesh(Spatial modelRoot) {
        List<Geometry> untaggedGeometries;
        if (modelRoot instanceof Geometry) {
            /*
             * To be consistent with createDynamicMeshShape() and others,
             * a nodeless model isn't checked for JME_PHYSICSIGNORE.
             */
            untaggedGeometries = new ArrayList<Geometry>(1);
            untaggedGeometries.add((Geometry) modelRoot);

        } else if (modelRoot instanceof Node) {
            untaggedGeometries = listUntaggedGeometries((Node) modelRoot, null);

        } else {
            throw new IllegalArgumentException(
                    "The model root must either be a Node or a Geometry!");
        }

        Collection<Geometry> includedGeometries
                = new ArrayList<>(untaggedGeometries.size());
        int totalIndices = 0;
        int totalVertices = 0;
        for (Geometry geometry : untaggedGeometries) {
            // Exclude any Geometry with a null/empty mesh.
            Mesh jmeMesh = geometry.getMesh();
            if (jmeMesh == null) {
                continue;
            }
            IndexBuffer indexBuffer = jmeMesh.getIndicesAsList();
            int numIndices = indexBuffer.size();
            if (numIndices == 0) {
                continue;
            }
            int numVertices = jmeMesh.getVertexCount();
            if (numVertices == 0) {
                continue;
            }

            includedGeometries.add(geometry);
            totalIndices += numIndices;
            totalVertices += numVertices;
        }

        IndexBuffer indexBuffer
                = IndexBuffer.createIndexBuffer(totalVertices, totalIndices);
        int totalFloats = numAxes * totalVertices;
        FloatBuffer positionBuffer = BufferUtils.createFloatBuffer(totalFloats);

        for (Geometry geometry : includedGeometries) {
            appendTriangles(geometry, modelRoot, positionBuffer, indexBuffer);
        }

        VertexBuffer.Format ibFormat = indexBuffer.getFormat();
        Buffer ibData = indexBuffer.getBuffer();
        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Index, MyMesh.vpt, ibFormat, ibData);
        result.setBuffer(VertexBuffer.Type.Position, numAxes, positionBuffer);

        return result;
    }

    /**
     * Calculate the Transform for a ChildCollisionShape relative to the
     * ancestor for which the shape is being generated.
     *
     * @param spatial (not null, unaffected)
     * @param modelRoot the ancestor for which the shape is being generated (not
     * null, unaffected)
     * @return a new Transform (not null)
     */
    private static Transform relativeTransform(
            Spatial spatial, Spatial modelRoot) {
        Transform result = new Transform();
        Spatial currentSpatial = spatial;
        while (currentSpatial != modelRoot) {
            MyMath.combine(result, currentSpatial.getLocalTransform(), result);
            currentSpatial = currentSpatial.getParent();
        }
        // Include the model root's scale but not its translation or rotation:
        Transform mrTransform = new Transform(); // TODO garbage
        mrTransform.setScale(modelRoot.getLocalScale());
        MyMath.combine(result, mrTransform, result);

        return result;
    }
}
