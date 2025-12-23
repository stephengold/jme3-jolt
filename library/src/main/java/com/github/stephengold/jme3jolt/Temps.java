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

import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.nio.DoubleBuffer;

/**
 * Thread-local temporary storage for internal use of the jme3-jolt library.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final class Temps {
    // *************************************************************************
    // fields

    /**
     * temporary storage for locations
     */
    final static ThreadLocal<DoubleBuffer> doubleBuffer
            = ThreadLocal.withInitial(() -> Jolt.newDirectDoubleBuffer(3));
    /**
     * temporary storage for converting quaternions
     */
    final static ThreadLocal<Quat> quat
            = ThreadLocal.withInitial(() -> new Quat());
    /**
     * temporary storage for converting quaternions
     */
    final static ThreadLocal<Quaternion> quaternion
            = ThreadLocal.withInitial(() -> new Quaternion());
    /**
     * temporary storage for converting vectors
     */
    final static ThreadLocal<RVec3> rvec3
            = ThreadLocal.withInitial(() -> new RVec3());
    /**
     * temporary storage for converting vectors
     */
    final static ThreadLocal<Vec3> vec3
            = ThreadLocal.withInitial(() -> new Vec3());
    /**
     * temporary storage for converting vectors
     */
    final static ThreadLocal<Vector3f> vector3f
            = ThreadLocal.withInitial(() -> new Vector3f());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Temps() {
    }
}
