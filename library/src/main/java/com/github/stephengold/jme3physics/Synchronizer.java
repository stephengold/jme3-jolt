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

import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;

/**
 * A control to link one or more physics objects to a subtree of the
 * scene graph and synchronize their positions.
 * <p>
 * If any of the linked physics objects is added to a simulation, then then all
 * of them must be added to that same simulation.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class Synchronizer extends AbstractControl {
    // *************************************************************************
    // fields

    /**
     * the simulation to which the control's physics objects are added, or
     * {@code null} if none of them are added to any simulation
     */
    private Simulator sim;
    // *************************************************************************
    // new protected methods

    /**
     * Access the simulation to which the control's physics objects are added,
     * if any.
     *
     * @return the pre-existing object, or {@code null} if none of them are
     * added to any simulation
     */
    protected Simulator findSim() {
        return sim;
    }

    /**
     * Alter which simulation the control's physics objects are added to.
     * <p>
     * Overriding methods should either create physics objects and add them to
     * the specified simulation, or (if {@code newSim == null}) remove them from
     * the simulation they're in.
     *
     * @param newSim the desired simulation, or {@code null} to remove
     */
    protected void setSim(Simulator newSim) {
        this.sim = newSim;
    }
    // *************************************************************************
    // AbstractControl methods

    /**
     * Clone this control for a different spatial. Obsolete since JME v3.1 .
     *
     * @param spatial (unused)
     * @return never
     * @throws UnsupportedOperationException always
     */
    @Override
    final public Synchronizer cloneForSpatial(Spatial spatial) {
        throw new UnsupportedOperationException(
                "cloneForSpatial() isn't implemented for "
                + getClass().getSimpleName());
    }

    /**
     * Render the control. Invoked once per viewport per frame, provided the
     * control is enabled and added to a scene. Should be invoked only by a
     * subclass or by {@code AbstractControl}.
     *
     * @param rm the render manager (unused)
     * @param vp the view port to render (unused)
     */
    @Override
    final protected void controlRender(RenderManager rm, ViewPort vp) {
        // do nothing
    }
}
