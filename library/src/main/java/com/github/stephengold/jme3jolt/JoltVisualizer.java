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

import com.github.stephengold.jme3physics.Visualizer;
import com.github.stephengold.joltjni.BodyIdVector;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.ConstraintRef;
import com.github.stephengold.joltjni.Constraints;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.TwoBodyConstraint;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.operator.Op;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstConstraint;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.Materials;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.debug.Arrow;
import java.nio.DoubleBuffer;
import java.util.HashMap;
import java.util.Map;
import jme3utilities.Validate;

/**
 * An appstate to visualize a Jolt physics simulation.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JoltVisualizer extends Visualizer {
    // *************************************************************************
    // constants

    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_Z}
     */
    final private static Vector3f unitZ = new Vector3f(0f, 0f, 1f);
    // *************************************************************************
    // fields

    /**
     * map each body's ID to an appstate-managed geometry
     */
    final private Map<Integer, Geometry> bodyGeometries = new HashMap<>();
    /**
     * map each body's ID to a shape summary
     */
    final private Map<Integer, ShapeSummary> bodyShapeSummary = new HashMap<>();
    /**
     * map each constraint to an appstate-managed blue-arrow geometry
     */
    final private Map<Long, Geometry> blueGeometries = new HashMap<>();
    /**
     * map each constraint to an appstate-managed green-arrow geometry
     */
    final private Map<Long, Geometry> greenGeometries = new HashMap<>();
    /**
     * map each constraint to an appstate-managed red-arrow geometry
     */
    final private Map<Long, Geometry> redGeometries = new HashMap<>();
    /**
     * material to visualize blue arrows
     */
    private Material blue;
    /**
     * material to visualize gray wireframes
     */
    private Material grayWire;
    /**
     * material to visualize green arrows
     */
    private Material green;
    /**
     * material to visualize magenta wireframes
     */
    private Material magentaWire;
    /**
     * material to visualize red arrows
     */
    private Material red;
    /**
     * material to visualize yellow wireframes
     */
    private Material yellowWire;
    /**
     * system of bodies and constraints
     */
    final private PhysicsSystem physicsSystem;
    /**
     * center-of-mass location of the first end of a constraint
     */
    final private RVec3 tmpCom1 = new RVec3();
    /**
     * center-of-mass location of the 2nd end of a constraint
     */
    final private RVec3 tmpCom2 = new RVec3();
    /**
     * pivot location of the first end of a constraint
     */
    final private RVec3 tmpPivot1 = new RVec3();
    /**
     * pivot location of the 2nd end of a constraint
     */
    final private RVec3 tmpPivot2 = new RVec3();
    // *************************************************************************
    // constructors

    /**
     * Instantiate an appstate that's enabled but not attached.
     *
     * @param system the physics system to visualize (not null, alias created)
     * @param parent where to attach the appstate's private scene-graph node
     * (not null)
     */
    JoltVisualizer(PhysicsSystem system, Node parent) {
        super("ShowJolt", parent);
        this.physicsSystem = system;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Initialize the materials used.
     *
     * @param manager the asset manager to use (not null)
     */
    @Override
    protected void setupMaterials(AssetManager manager) {
        this.blue = new Material(manager, Materials.UNSHADED);
        blue.getAdditionalRenderState().setWireframe(true);
        blue.setColor("Color", ColorRGBA.Blue);

        this.grayWire = new Material(manager, Materials.UNSHADED);
        grayWire.getAdditionalRenderState().setWireframe(true);
        grayWire.setColor("Color", ColorRGBA.LightGray);

        this.green = new Material(manager, Materials.UNSHADED);
        green.getAdditionalRenderState().setWireframe(true);
        green.setColor("Color", ColorRGBA.Green);

        this.magentaWire = new Material(manager, Materials.UNSHADED);
        magentaWire.getAdditionalRenderState().setWireframe(true);
        magentaWire.setColor("Color", ColorRGBA.Magenta);

        this.red = new Material(manager, Materials.UNSHADED);
        red.getAdditionalRenderState().setWireframe(true);
        red.setColor("Color", ColorRGBA.Red);

        this.yellowWire = new Material(manager, Materials.UNSHADED);
        yellowWire.getAdditionalRenderState().setWireframe(true);
        yellowWire.setColor("Color", ColorRGBA.Yellow);
    }
    // *************************************************************************
    // AbstractAppState methods

    /**
     * Initialize the appstate.
     *
     * @param stateManager the state manager (not null)
     * @param app the application (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        AssetManager assetManager = app.getAssetManager();
        setupMaterials(assetManager);
    }

    /**
     * Update the appstate. This method will be invoked during every update when
     * the appstate is both attached and enabled.
     *
     * @param tpf the time interval between update calls (in seconds)
     */
    @Override
    public void update(float tpf) {
        assert Validate.nonNegative(tpf, "time interval");

        Node node = getShowNode();
        node.detachAllChildren();

        BodyIdVector bodyIds = new BodyIdVector();
        physicsSystem.getBodies(bodyIds);
        Constraints constraints = physicsSystem.getConstraints();
        BodyInterface bi = physicsSystem.getBodyInterface();

        int numBodies = bodyIds.size();
        for (int i = 0; i < numBodies; ++i) {
            int bodyId = bodyIds.get(i);
            assert bi.isAdded(bodyId);
            BodyLockRead lock = new BodyLockRead(
                    physicsSystem.getBodyLockInterface(), bodyId);
            assert lock.succeeded();
            ConstBody body = lock.getBody();
            if (body.isSoftBody()) {
                // TODO
                continue;
            } else {
                updateRigidBody(body, bodyId);
            }
            lock.releaseLock();
        }

        int numConstraints = constraints.size();
        for (int i = 0; i < numConstraints; ++i) {
            ConstraintRef ref = constraints.get(i);
            ConstConstraint c = ref.getPtr();
            assert physicsSystem.containsConstraint(c);

            if (c instanceof TwoBodyConstraint) {
                updateTwoBodyConstraint((TwoBodyConstraint) c);
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Position an arrow geometry so that its tail and tip and in the specified
     * locations.
     *
     * @param arrowGeometry the geometry to position (not null, modified)
     * @param tail the desired tail location (not null, unaffected)
     * @param tip the desired tip location (not null, unaffected)
     */
    private void positionArrow(
            Geometry arrowGeometry, RVec3Arg tail, RVec3Arg tip) {
        arrowGeometry.setLocalTranslation(tail.x(), tail.y(), tail.z());

        RVec3Arg offset = Op.minus(tip, tail);
        float length = (float) offset.length();
        arrowGeometry.setLocalScale(length);

        if (length > 0f) {
            Vec3 direction = offset.toVec3();
            direction.scaleInPlace(1f / length);
            QuatArg rotation = Quat.sFromTo(Vec3.sAxisZ(), direction);
            Quaternion q = new Quaternion(rotation.getX(), rotation.getY(),
                    rotation.getZ(), rotation.getW());
            arrowGeometry.setLocalRotation(q);
        }
    }

    /**
     * Update the visualization of the specified rigid body.
     *
     * @param body the body being visualized (not null, unaffected)
     * @param bodyId the ID of the body being visualized
     */
    private void updateRigidBody(ConstBody body, int bodyId) {
        Geometry geometry = bodyGeometries.get(bodyId);
        if (geometry == null) {
            geometry = new Geometry();
            bodyGeometries.put(bodyId, geometry);
        }
        Node node = getShowNode();
        node.attachChild(geometry);

        // Update the geometry's material:
        if (body.isSensor()) {
            geometry.setMaterial(yellowWire);
        } else if (body.isDynamic() && body.isActive()) {
            geometry.setMaterial(magentaWire);
        } else {
            geometry.setMaterial(grayWire);
        }

        // Update the geometry's transform:
        RVec3 joltLocation = Temps.rvec3.get();
        Quat joltOrientation = Temps.quat.get();
        body.getPositionAndRotation(joltLocation, joltOrientation);
        Quaternion orientation = Temps.quaternion.get();
        orientation.set(joltOrientation.getX(), joltOrientation.getY(),
                joltOrientation.getZ(), joltOrientation.getW());
        geometry.setLocalRotation(orientation);
        geometry.setLocalTranslation(
                joltLocation.x(), joltLocation.y(), joltLocation.z());

        // Update the body's shape summary and the geometry's mesh:
        ConstShape shape = body.getShape();
        ShapeSummary summary = bodyShapeSummary.get(bodyId);
        if (summary == null || !summary.matches(shape)) {
            summary = new ShapeSummary(shape);
            bodyShapeSummary.put(bodyId, summary);
            Mesh mesh = new ShapeMesh(shape);
            geometry.setMesh(mesh);
        }
    }

    /**
     * Update the visualization of the specified two-body constraint.
     *
     * @param tbc the constraint being visualized (not null, unaffected)
     */
    private void updateTwoBodyConstraint(TwoBodyConstraint tbc) {
        DoubleBuffer doubleBuffer = Temps.doubleBuffer.get();
        tbc.getBody1PivotLocation(doubleBuffer);
        tmpPivot1.set(doubleBuffer);

        tbc.getBody2PivotLocation(doubleBuffer);
        tmpPivot2.set(doubleBuffer);

        ConstBody body1 = tbc.getBody1();
        body1.getCenterOfMassPosition(doubleBuffer);
        tmpCom1.set(doubleBuffer);

        ConstBody body2 = tbc.getBody2();
        body2.getCenterOfMassPosition(doubleBuffer);
        tmpCom2.set(doubleBuffer);

        // blue arrow from pivot1 to pivot2:
        long constraintVa = tbc.va();
        Geometry blueGeometry = blueGeometries.get(constraintVa);
        if (blueGeometry == null) {
            Arrow mesh = new Arrow(unitZ);
            blueGeometry = new Geometry("", mesh);
            blueGeometry.setMaterial(blue);
            blueGeometries.put(constraintVa, blueGeometry);
        }
        Node node = getShowNode();
        node.attachChild(blueGeometry);
        positionArrow(blueGeometry, tmpPivot1, tmpPivot2);

        // green arrow from com1 to pivot1:
        Geometry greenGeometry = greenGeometries.get(constraintVa);
        if (greenGeometry == null) {
            Arrow mesh = new Arrow(unitZ);
            greenGeometry = new Geometry("", mesh);
            greenGeometry.setMaterial(green);
            greenGeometries.put(constraintVa, greenGeometry);
        }
        node.attachChild(greenGeometry);
        positionArrow(greenGeometry, tmpCom1, tmpPivot1);

        // red arrow from com2 to pivot2:
        Geometry redGeometry = redGeometries.get(constraintVa);
        if (redGeometry == null) {
            Arrow mesh = new Arrow(unitZ);
            redGeometry = new Geometry("", mesh);
            redGeometry.setMaterial(red);
            redGeometries.put(constraintVa, redGeometry);
        }
        node.attachChild(redGeometry);
        positionArrow(greenGeometry, tmpCom2, tmpPivot2);
    }
}
