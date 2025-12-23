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

import com.github.stephengold.jme3physics.Simulator;
import com.github.stephengold.jme3physics.Synchronizer;
import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.BodyLockWrite;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.ObjectStreamIn;
import com.github.stephengold.joltjni.ObjectStreamOut;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.ShapeSettings;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EStreamType;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstBodyCreationSettings;
import com.github.stephengold.joltjni.readonly.ConstBodyLockInterface;
import com.github.stephengold.joltjni.readonly.ConstShapeSettings;
import com.github.stephengold.joltjni.std.StringStream;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.util.clone.Cloner;
import java.io.IOException;
import jme3utilities.MySpatial;
import jme3utilities.Validate;

/**
 * Synchroize a Jolt rigid body with a spatial.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RigidSynchronizer extends Synchronizer {
    // *************************************************************************
    // constants

    /**
     * tag for serializing the isSpatialDriven field
     */
    final private static String tagIsSpatialDriven = "isSpatialDriven";
    /**
     * tag for serializing uncooked body-creation settings
     */
    final private static String tagUncookedBcs = "uncookedBcs";
    // *************************************************************************
    // fields

    /**
     * settings for body creation (possibly cooked), or {@code null} if the body
     * is added to a system
     */
    private BodyCreationSettings bcs;
    /**
     * how the body is driven when it's in kinematic mode
     */
    private boolean isSpatialDriven;
    /**
     * shape settings (not null)
     */
    private ConstShapeSettings shapeSettings;
    /**
     * ID of the physics body, or {@code cInvalidBodyId} if the body is not
     * added to a system
     */
    private int bodyId;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    protected RigidSynchronizer() {
    }

    /**
     * Instantiate an enabled synchronizer that's not added to any physics
     * space, using the specified settings.
     *
     * @param settings the raw body-creation settings to use (not null,
     * unaffected)
     */
    public RigidSynchronizer(ConstBodyCreationSettings settings) {
        ConstShapeSettings ss = settings.getShapeSettings();
        Validate.require(ss != null, "raw settings");

        this.shapeSettings = ShapeSettings.cloneShapeSettings(ss);
        this.bcs = new BodyCreationSettings(settings);
        this.bodyId = Jolt.cInvalidBodyId;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether the body could be made kinematic or dynamic.
     *
     * @return {@code true} if possible, otherwise {@code false}
     */
    public boolean canBeKinematicOrDynamic() {
        boolean result;

        JoltSimulator simulator = findSim();
        if (simulator == null) {
            result = bcs.getAllowDynamicOrKinematic();
        } else {
            PhysicsSystem system = simulator.getPhysicsSystem();
            ConstBodyLockInterface bli = system.getBodyLockInterface();
            BodyLockRead lock = new BodyLockRead(bli, bodyId);
            if (lock.succeeded()) {
                ConstBody body = lock.getBody();
                result = body.canBeKinematicOrDynamic();
            } else {
                result = false;
            }
            lock.close();
        }

        return result;
    }

    /**
     * Return uncooked body-creation settings.
     *
     * @return a new settings object with non-null shape settings
     */
    public BodyCreationSettings copyUncookedBcs() {
        BodyCreationSettings result;

        JoltSimulator simulator = findSim();
        if (simulator == null) {
            result = new BodyCreationSettings(bcs);
        } else {
            PhysicsSystem system = simulator.getPhysicsSystem();
            ConstBodyLockInterface bli = system.getBodyLockInterface();
            BodyLockRead lock = new BodyLockRead(bli, bodyId);
            assert lock.succeeded();
            ConstBody body = lock.getBody();
            result = body.getBodyCreationSettings();
            lock.releaseLock();
            result.setShapeSettings(shapeSettings);
        }

        assert result.getShapeSettings() != null;
        return result;
    }
    // *************************************************************************
    // Synchronizer methods

    /**
     * Callback from {@link com.jme3.util.clone.Cloner} to convert this
     * shallow-cloned synchronizer into a deep-cloned one, using the specified
     * cloner and original to resolve copied fields.
     * <p>
     * The clone will not be added to any physics system, even if the original
     * has been.
     *
     * @param cloner the cloner that's cloning this synchronizer (not null)
     * @param original the instance from which this synchronizer was
     * shallow-cloned (not null, unaffected)
     */
    @Override
    public void cloneFields(Cloner cloner, Object original) {
        this.bodyId = Jolt.cInvalidBodyId;
        this.bcs = copyUncookedBcs();
        this.shapeSettings = ShapeSettings.cloneShapeSettings(shapeSettings);
        super.setSim(null);
    }

    /**
     * Update the control. Invoked once per frame during the logical-state
     * update, provided the control is enabled and added to a scene. Should be
     * invoked only by a subclass or by AbstractControl.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    protected void controlUpdate(float tpf) {
        JoltSimulator simulator = findSim();
        if (simulator == null) {
            return;
        }

        PhysicsSystem system = simulator.getPhysicsSystem();
        ConstBodyLockInterface bli = system.getBodyLockInterface();
        BodyLockWrite lock = new BodyLockWrite(bli, bodyId);
        assert lock.succeeded();
        Body body = lock.getBody();

        EMotionType motionType = body.getMotionType();
        if (motionType == EMotionType.Kinematic && isSpatialDriven) {
            Vector3f location = Temps.vector3f.get();
            location.set(spatial.getWorldTranslation());

            Quaternion orientation = Temps.quaternion.get();
            orientation.set(spatial.getWorldRotation());

            RVec3 joltLocation = Temps.rvec3.get();
            joltLocation.set(location.x, location.y, location.z);

            float qw = orientation.getW();
            float qx = orientation.getX();
            float qy = orientation.getY();
            float qz = orientation.getZ();
            Quat joltOrientation = Temps.quat.get();
            joltOrientation.set(qx, qy, qz, qw);

            body.moveKinematic(joltLocation, joltOrientation, tpf);

        } else { // physics-driven spatial
            RVec3 joltLocation = Temps.rvec3.get();
            Quat joltOrientation = Temps.quat.get();
            body.getPositionAndRotation(joltLocation, joltOrientation);

            Vector3f location = Temps.vector3f.get();
            location.set(joltLocation.x(), joltLocation.y(), joltLocation.z());

            float qw = joltOrientation.getW();
            float qx = joltOrientation.getX();
            float qy = joltOrientation.getY();
            float qz = joltOrientation.getZ();
            Quaternion orientation = Temps.quaternion.get();
            orientation.set(qx, qy, qz, qw);

            MySpatial.setWorldLocation(spatial, location);
            MySpatial.setWorldOrientation(spatial, orientation);
        }

        lock.releaseLock();
    }

    /**
     * Access the simulator to which the synchronizer's physics objects are
     * added, if any.
     *
     * @return the pre-existing object, or {@code null} if none of them are
     * added to any simulator
     */
    @Override
    final public JoltSimulator findSim() {
        JoltSimulator result = (JoltSimulator) super.findSim();
        return result;
    }

    /**
     * Create a shallow clone for the JME cloner. Note that the cloned object
     * won't be added to any system, even if the original was.
     *
     * @return a new instance without a native object assigned
     */
    @Override
    public RigidSynchronizer jmeClone() {
        try {
            RigidSynchronizer clone = (RigidSynchronizer) clone();
            return clone;
        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * De-serialize this synchronizer from the specified importer, for example
     * when loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        InputCapsule capsule = importer.getCapsule(this);

        this.isSpatialDriven = capsule.readBoolean(tagIsSpatialDriven, false);

        String ubcsString = capsule.readString(tagUncookedBcs, null);
        assert ubcsString != null;
        StringStream ubcsStream = new StringStream(ubcsString);
        BodyCreationSettings[] storeBcs = new BodyCreationSettings[1];
        boolean success = ObjectStreamIn.sReadObject(ubcsStream, storeBcs);
        assert success;
        this.bcs = storeBcs[0];

        ConstShapeSettings ss = bcs.getShapeSettings();
        assert ss != null;
        this.shapeSettings = ShapeSettings.cloneShapeSettings(ss);
    }

    /**
     * Create the body and add it to the specified simulator, or remove it.
     *
     * @param newSim the desired simulator, or {@code null} to remove the body
     * from any simulator it's added to
     */
    @Override
    protected void setSim(Simulator newSim) {
        JoltSimulator oldSim = findSim();
        if (newSim == oldSim) { // no change
            return;
        }

        if (oldSim != null) { // remove body from the old simulator
            this.bcs = copyUncookedBcs();

            PhysicsSystem system = oldSim.getPhysicsSystem();
            BodyInterface bi = system.getBodyInterface();
            bi.removeBody(bodyId);

            this.bodyId = Jolt.cInvalidBodyId;
            super.setSim(null);
            oldSim.addRemove();
        }

        assert bodyId == Jolt.cInvalidBodyId;
        assert bcs != null;
        assert findSim() == null;

        if (newSim != null) { // add body to the new simulator
            super.setSim(newSim);
            JoltSimulator joltSim = (JoltSimulator) newSim;
            joltSim.addRemove();

            PhysicsSystem system = joltSim.getPhysicsSystem();
            BodyInterface bi = system.getBodyInterface();
            EMotionType motionType = bcs.getMotionType();
            EActivation activation = (motionType == EMotionType.Static)
                    ? EActivation.DontActivate : EActivation.Activate;

            this.bodyId = bi.createAndAddBody(bcs, activation);
            this.bcs = null;
        }
    }

    /**
     * Serialize this synchronizer to the specified exporter, for example when
     * saving to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        OutputCapsule capsule = exporter.getCapsule(this);

        capsule.write(isSpatialDriven, tagIsSpatialDriven, false);

        StringStream ubcsStream = new StringStream();
        ConstBodyCreationSettings uncookedBcs = copyUncookedBcs();
        boolean success = ObjectStreamOut.sWriteObject(
                ubcsStream, EStreamType.Binary, uncookedBcs);
        assert success;
        String ubcsString = ubcsStream.str();
        capsule.write(ubcsString, tagUncookedBcs, null);
        assert uncookedBcs.getShapeSettings() != null;
    }
}
