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
import com.github.stephengold.jme3physics.Visualizer;
import com.github.stephengold.joltjni.BroadPhaseLayerInterface;
import com.github.stephengold.joltjni.BroadPhaseLayerInterfaceTable;
import com.github.stephengold.joltjni.JobSystem;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.ObjectLayerPairFilterTable;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilter;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilterTable;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorMalloc;
import com.jme3.scene.Node;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * An appstate to create and simulate a Jolt-JNI {@code PhysicsSystem}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class JoltSimulator extends Simulator {
    // *************************************************************************
    // constants

    /**
     * default maximum number of bodies
     */
    final private static int defaultMaxBodies = 5_000;
    /**
     * default number of broadphase layers
     */
    final private static int defaultNumBpLayers = 2;
    // *************************************************************************
    // fields

    /**
     * total collision-object adds and removes since the previous broadphase
     * optimization
     */
    private int addRemoveCount;
    /**
     * number of adds and removes to trigger broadphase optimization
     */
    private int bpoThreshold = 5;
    /**
     * schedule simulation jobs (not {@code null})
     */
    final private JobSystem jobSystem;
    /**
     * system of bodies and constraints to simulate (not {@code null})
     */
    final private PhysicsSystem physicsSystem;
    /**
     * allocate temporary memory for simulation (not {@code null})
     */
    final private TempAllocator tempAllocator;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an appstate that's enabled but not attached.
     */
    public JoltSimulator() {
        this(defaultMaxBodies, defaultNumBpLayers, Jolt.cMaxPhysicsJobs,
                Jolt.cMaxPhysicsBarriers, defaultNumThreads());
    }

    /**
     * Instantiate an appstate that's enabled but not attached.
     *
     * @param maxBodies the maximum number of bodies (&ge;1)
     * @param numBpLayers the number of broadphase layers (1 or 2)
     * @param maxJobs the maximum number of jobs the job system can allocate
     * (&gt;1)
     * @param maxBarriers the maximum number of barriers the job system can
     * allocate
     * @param numThreads the number of worker threads to start (&ge;0) or -1 to
     * autodetect
     */
    public JoltSimulator(int maxBodies, int numBpLayers, int maxJobs,
            int maxBarriers, int numThreads) {
        super("JoltSim");

        if (!JoltLibraryLoader.isLoaded()) {
            JoltLibraryLoader.load();
        }

        this.physicsSystem = createSystem(maxBodies, numBpLayers);
        this.jobSystem
                = new JobSystemThreadPool(maxJobs, maxBarriers, numThreads);
        this.tempAllocator = new TempAllocatorMalloc();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Increment the add-remove count.
     */
    void addRemove() {
        ++addRemoveCount;
    }

    /**
     * Create a visualization appstate for the physics system.
     *
     * @param parentNode where to attach the appstate's private scene-graph node
     * (not null)
     * @return a new object
     */
    @Override
    public Visualizer createShowPhysics(Node parentNode) {
        Visualizer result = new JoltVisualizer(physicsSystem, parentNode);
        return result;
    }

    /**
     * Return the number of adds-plus-removes to trigger broadphase
     * optimization.
     *
     * @return the count
     */
    public int getBpoThreshold() {
        return bpoThreshold;
    }

    /**
     * Access the internal {@code PhysicsSystem}.
     *
     * @return the pre-existing object
     */
    final PhysicsSystem getPhysicsSystem() {
        return physicsSystem;
    }

    /**
     * Alter the number of adds and removes to trigger broadphase optimization.
     *
     * @param numControls the desired threshold value
     * @return the modified appstate, for chaining
     */
    public JoltSimulator setBpoThreshold(int numControls) {
        Validate.nonNegative(numControls, "number of controls");
        this.bpoThreshold = numControls;
        return this;
    }
    // *************************************************************************
    // SimPhysics methods

    /**
     * Single-step the physics simulation.
     *
     * @param deltaTime the number of seconds to simulate (&ge;0)
     */
    @Override
    protected void singleStep(float deltaTime) {
        processQueuedControls();
        if (addRemoveCount >= bpoThreshold) {
            physicsSystem.optimizeBroadPhase();
            this.addRemoveCount = 0;

        } else if (addRemoveCount > 0) {
            ++addRemoveCount; // to ensure optimization will eventually occur
        }

        // Single-step the physics system:
        preStep(deltaTime);
        int collisionSteps = 1;
        physicsSystem.update(
                deltaTime, collisionSteps, tempAllocator, jobSystem);
        postStep(deltaTime);

    }
    // *************************************************************************
    // private methods

    /**
     * Generate a mapping from object layers to broadphase layers.
     *
     * @param numBpLayers the desired number of broadphase layers (1 or 2)
     * @return a new object
     */
    private static BroadPhaseLayerInterface createLayerMap(int numBpLayers) {
        BroadPhaseLayerInterfaceTable result
                = new BroadPhaseLayerInterfaceTable(
                        ObjectLayers.count, numBpLayers);
        if (numBpLayers == 2) {
            /*
             * Identify non-moving objects sooner
             * by mapping them to a separate broadphase layer:
             */
            int bpLayerMoving = 0;
            int bpLayerNonMoving = 1;
            result.mapObjectToBroadPhaseLayer(
                    ObjectLayers.moving, bpLayerMoving);
            result.mapObjectToBroadPhaseLayer(
                    ObjectLayers.moving, bpLayerNonMoving);

        } else {
            assert numBpLayers == 1 : "numBpLayers = " + numBpLayers;
            /*
             * Map all objects to a single broadphase layer,
             * which is simpler but less efficient:
             */
            int bpLayerAll = 0;
            result.mapObjectToBroadPhaseLayer(ObjectLayers.moving, bpLayerAll);
            result.mapObjectToBroadPhaseLayer(
                    ObjectLayers.nonMoving, bpLayerAll);
        }

        return result;
    }

    /**
     * Create a generic PhysicsSystem with the specified parameters.
     *
     * @param maxBodies the maximum number of bodies (&ge;1)
     * @param numBpLayers the number of broadphase layers (1 or 2)
     * @return a new object
     */
    private static PhysicsSystem createSystem(int maxBodies, int numBpLayers) {
        Validate.positive(maxBodies, "maximum number of bodies");
        Validate.inRange(numBpLayers, "number of BP layers", 1, 2);
        /*
         * The number of object layers in jme3-jolt is fixed at 2:
         * one for moving objects and one for non-moving ones.
         *
         * Configure an object-layer pair filter to ignore collisions
         * between non-moving objects:
         */
        ObjectLayerPairFilterTable ovoFilter
                = new ObjectLayerPairFilterTable(ObjectLayers.count);
        ovoFilter.enableCollision(ObjectLayers.moving, ObjectLayers.moving);
        ovoFilter.enableCollision(ObjectLayers.moving, ObjectLayers.nonMoving);

        BroadPhaseLayerInterface layerMap = createLayerMap(numBpLayers);
        /*
         * Pre-compute the rules for colliding object layers
         * with broadphase layers:
         */
        ObjectVsBroadPhaseLayerFilter ovbFilter
                = new ObjectVsBroadPhaseLayerFilterTable(
                        layerMap, numBpLayers, ovoFilter, ObjectLayers.count);

        PhysicsSystem result = new PhysicsSystem();
        int numBodyMutexes = 0; // 0 means "use the default value"
        long maxBodiesLong = maxBodies;
        long possiblePairs = maxBodiesLong * (maxBodiesLong - 1) / 2;
        int maxBodyPairs = (int) Math.min(possiblePairs, 60_000);
        maxBodyPairs = Math.max(3, maxBodyPairs);
        int maxContacts = 6 * (maxBodies + 6);

        result.init(maxBodies, numBodyMutexes, maxBodyPairs, maxContacts,
                layerMap, ovbFilter, ovoFilter);

        return result;
    }

    /**
     * Return the recommended number of worker threads to use.
     *
     * @return the count (&ge;1, &le;64)
     */
    private static int defaultNumThreads() {
        int numCpus = Runtime.getRuntime().availableProcessors();
        int result = (int) Math.floor(0.9 * numCpus);
        result = MyMath.clamp(result, 1, 64);

        return result;
    }
}
