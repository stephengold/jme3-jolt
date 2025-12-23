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
package com.github.stephengold.jme3physics;

import com.jme3.app.state.AbstractAppState;
import com.jme3.math.FastMath;
import com.jme3.scene.Node;
import com.jme3.util.SafeArrayList;
import java.util.Collection;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import jme3utilities.Validate;

/**
 * An appstate to create and simulate physics in jMonkeyEngine.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class Simulator extends AbstractAppState {
    // *************************************************************************
    // fields

    /**
     * list of registered post-step listeners
     */
    final private Collection<PostStepListener> postStepListeners
            = new SafeArrayList<>(PostStepListener.class);
    /**
     * list of registered pre-step listeners
     */
    final private Collection<PreStepListener> preStepListeners
            = new SafeArrayList<>(PreStepListener.class);
    /**
     * maximum simulated time per step for {@code maxStepsPerUpdate==0} (in
     * seconds, &gt;0)
     */
    private float maxTimePerStep = 0.1f;
    /**
     * simulation lag for {@code maxStepsPerUpdate>0} (in seconds, &ge;0)
     */
    private float physicsLag;
    /**
     * preferred simulated time per step for {@code maxStepsPerUpdate>0} (in
     * seconds, &ge;0)
     */
    private float preferredTimePerStep = 1 / 60f;
    /**
     * maximum number of simulation steps per update (&gt;0) or 0 for a variable
     * time step
     */
    private int maxStepsPerUpdate = 4;
    /**
     * controls that are queued up to be added to the {@code PhysicsSystem}
     */
    final private Queue<Synchronizer> addControls
            = new ConcurrentLinkedQueue<>();
    /**
     * controls that are queued up to be removed from the {@code PhysicsSystem}
     */
    final private Queue<Synchronizer> removeControls
            = new ConcurrentLinkedQueue<>();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a simulator with the specified ID.
     *
     * @param id the desired app-state identifier
     */
    protected Simulator(String id) {
        super(id);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Create a visualization appstate for the physics system.
     *
     * @param parentNode where to attach the appstate's private scene-graph node
     * (not null)
     * @return a new object
     */
    abstract public Visualizer createShowPhysics(Node parentNode);

    /**
     * Return the maximum number of simulation steps per update (&gt;0) or 0 for
     * a variable time step.
     *
     * @return the count or zero
     */
    public int getMaxStepsPerUpdate() {
        return maxStepsPerUpdate;
    }

    /**
     * Return maximum simulated time per step for {@code maxStepsPerUpdate==0}.
     *
     * @return the time per step (in seconds)
     */
    public float getMaxTimePerStep() {
        return maxTimePerStep;
    }

    /**
     * Return preferred simulated time per step for {@code maxStepsPerUpdate>0}.
     *
     * @return the count
     */
    public float getPreferredTimePerStep() {
        return preferredTimePerStep;
    }

    /**
     * Process any queued controls and update the add/remove actions count. This
     * method is invoked automatically, but may also be explicitly invoked by
     * applications.
     */
    public void processQueuedControls() {
        Synchronizer addControl;
        while (true) {
            addControl = addControls.poll();
            if (addControl == null) {
                break;
            }
            addControl.setSim(this);
        }

        Synchronizer removeControl;
        while (true) {
            removeControl = removeControls.poll();
            if (removeControl == null) {
                break;
            }
            removeControl.setSim(null);
        }

        // Extra polling because controls might be queued asynchronously.
        do {
            addControl = addControls.poll();
            if (addControl != null) {
                addControl.setSim(this);
            }
            removeControl = removeControls.poll();
            if (removeControl != null) {
                removeControl.setSim(null);
            }
        } while (addControl != null || removeControl != null);
    }

    /**
     * Queue the specified physics control for addition.
     *
     * @param control the control to add (not null)
     */
    public void queueForAddition(Synchronizer control) {
        addControls.add(control);
    }

    /**
     * Queue the specified physics control for removal.
     *
     * @param control the control to remove (not null)
     */
    public void queueForRemoval(Synchronizer control) {
        removeControls.add(control);
    }

    /**
     * Alter the maximum number of simulated steps per update.
     *
     * @param numSteps the desired limit, or 0 for a variable time step
     * @return the modified appstate, for chaining
     */
    public Simulator setMaxStepsPerUpdate(int numSteps) {
        Validate.nonNegative(numSteps, "number of steps");
        this.maxStepsPerUpdate = numSteps;
        return this;
    }

    /**
     * Alter the maximum simulated time per step for
     * {@code maxStepsPerUpdate==0}.
     *
     * @param timePerStep the desired limit (in seconds)
     * @return the modified appstate, for chaining
     */
    public Simulator setMaxTimePerStep(float timePerStep) {
        Validate.positive(timePerStep, "time per step");
        this.maxTimePerStep = timePerStep;
        return this;
    }

    /**
     * Alter the preferred simulated time per step.
     *
     * @param timePerStep the desired time step (in simulated seconds)
     * @return the modified appstate, for chaining
     */
    public Simulator setPreferredTimePerStep(float timePerStep) {
        Validate.positive(timePerStep, "time per step");
        this.preferredTimePerStep = timePerStep;
        return this;
    }
    // *************************************************************************
    // protected methods

    /**
     * Advance the physics simulation by the specified amount.
     *
     * @param timeInterval the number of seconds to simulate (&ge;0)
     * @param maxSteps the maximum number of simulation steps of the preferred
     * size, or zero for a single step of {@code timeInterval} seconds
     */
    protected void advancePhysics(float timeInterval, int maxSteps) {
        float timePerStep;
        int numSubSteps;
        if (maxSteps == 0) { // a single step of 'timeInterval' seconds
            timePerStep = timeInterval;
            numSubSteps = 1;

        } else { // up to 'maxSteps' steps of the preferred size
            timePerStep = preferredTimePerStep;

            float timeSinceStep = physicsLag + timeInterval;
            numSubSteps = (int) FastMath.floor(timeSinceStep / timePerStep);
            assert numSubSteps >= 0 : numSubSteps;
            this.physicsLag = timeSinceStep - numSubSteps * timePerStep;
            assert physicsLag >= 0f : physicsLag;

            if (numSubSteps > maxSteps) {
                numSubSteps = maxSteps;
            }
        }

        for (int i = 0; i < numSubSteps; ++i) {
            singleStep(timePerStep);
        }
    }

    /**
     * Invoked just after the physics is stepped.
     *
     * @param timeStep the amount of time to be simulated (in seconds, &ge;0)
     */
    protected void postStep(float timeStep) {
        for (PostStepListener listener : postStepListeners) {
            listener.postStep(this, timeStep);
        }
    }

    /**
     * Invoked just before the physics is stepped.
     *
     * @param timeStep the amount of time that was simulated (in seconds, &ge;0)
     */
    protected void preStep(float timeStep) {
        for (PreStepListener listener : preStepListeners) {
            listener.preStep(this, timeStep);
        }
    }

    /**
     * Single-step the physics simulation.
     *
     * @param deltaTime the number of seconds to simulate (&ge;0)
     */
    abstract protected void singleStep(float deltaTime);
    // *************************************************************************
    // AbstractAppState methods

    /**
     * Update the appstate. This method will be invoked during every update when
     * the appstate is both attached and enabled.
     *
     * @param tpf the time interval between update calls (in seconds)
     */
    @Override
    public void update(float tpf) {
        assert Validate.nonNegative(tpf, "time interval");

        float timeInterval;
        if (maxStepsPerUpdate == 0) {
            timeInterval = Math.min(tpf, maxTimePerStep);
        } else {
            timeInterval = tpf;
            assert maxStepsPerUpdate > 0 : maxStepsPerUpdate;
        }
        advancePhysics(timeInterval, maxStepsPerUpdate);
    }
}
