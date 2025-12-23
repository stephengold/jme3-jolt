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

import com.jme3.app.Application;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.scene.Node;

/**
 * An appstate to visualize a physics simulation in JMonkeyEngine.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class Visualizer extends AbstractAppState {
    // *************************************************************************
    // fields

    /**
     * private scene-graph node, to which visualization geometries get attached
     */
    final private Node showNode;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a visualizer with the specified ID and parent node.
     *
     * @param id the desired app-state identifier
     * @param parent the node to use when attaching spatials to the scene graph
     */
    protected Visualizer(String id, Node parent) {
        super(id);

        this.showNode = new Node("show physics");
        parent.attachChild(showNode);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the node to which visualization geometries get attached.
     *
     * @return the pre-existing object
     */
    final protected Node getShowNode() {
        return showNode;
    }

    /**
     * Initialize the materials used.
     *
     * @param manager the asset manager to use (not null)
     */
    abstract protected void setupMaterials(AssetManager manager);
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
}
