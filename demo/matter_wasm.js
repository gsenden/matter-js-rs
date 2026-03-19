/* @ts-self-types="./matter_wasm.d.ts" */

export class PhysicsEngine {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        PhysicsEngineFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_physicsengine_free(ptr, 0);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} radius
     * @param {boolean} is_static
     * @returns {number}
     */
    addCircle(x, y, radius, is_static) {
        const ret = wasm.physicsengine_addCircle(this.__wbg_ptr, x, y, radius, is_static);
        return ret >>> 0;
    }
    /**
     * @param {number} body_a
     * @param {number} body_b
     * @param {number} stiffness
     * @returns {number}
     */
    addConstraint(body_a, body_b, stiffness) {
        const ret = wasm.physicsengine_addConstraint(this.__wbg_ptr, body_a, body_b, stiffness);
        return ret >>> 0;
    }
    /**
     * @param {number} body_a
     * @param {number} world_x
     * @param {number} world_y
     * @param {number} stiffness
     * @returns {number}
     */
    addPinConstraint(body_a, world_x, world_y, stiffness) {
        const ret = wasm.physicsengine_addPinConstraint(this.__wbg_ptr, body_a, world_x, world_y, stiffness);
        return ret >>> 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} sides
     * @param {number} radius
     * @param {boolean} is_static
     * @returns {number}
     */
    addPolygon(x, y, sides, radius, is_static) {
        const ret = wasm.physicsengine_addPolygon(this.__wbg_ptr, x, y, sides, radius, is_static);
        return ret >>> 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} width
     * @param {number} height
     * @param {boolean} is_static
     * @returns {number}
     */
    addRectangle(x, y, width, height, is_static) {
        const ret = wasm.physicsengine_addRectangle(this.__wbg_ptr, x, y, width, height, is_static);
        return ret >>> 0;
    }
    /**
     * @param {number} body_id
     * @param {number} px
     * @param {number} py
     * @param {number} fx
     * @param {number} fy
     */
    applyForce(body_id, px, py, fx, fy) {
        wasm.physicsengine_applyForce(this.__wbg_ptr, body_id, px, py, fx, fy);
    }
    /**
     * @param {number} x
     * @param {number} y
     * @returns {number}
     */
    bodyAtPoint(x, y) {
        const ret = wasm.physicsengine_bodyAtPoint(this.__wbg_ptr, x, y);
        return ret;
    }
    endDrag() {
        wasm.physicsengine_endDrag(this.__wbg_ptr);
    }
    /**
     * @returns {number}
     */
    getBodyCount() {
        const ret = wasm.physicsengine_getBodyCount(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @param {number} mouse_x
     * @param {number} mouse_y
     */
    moveDrag(mouse_x, mouse_y) {
        wasm.physicsengine_moveDrag(this.__wbg_ptr, mouse_x, mouse_y);
    }
    constructor() {
        const ret = wasm.physicsengine_new();
        this.__wbg_ptr = ret >>> 0;
        PhysicsEngineFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} scale
     */
    setGravity(x, y, scale) {
        wasm.physicsengine_setGravity(this.__wbg_ptr, x, y, scale);
    }
    /**
     * @param {number} body_id
     * @param {number} x
     * @param {number} y
     */
    setPosition(body_id, x, y) {
        wasm.physicsengine_setPosition(this.__wbg_ptr, body_id, x, y);
    }
    /**
     * @param {number} body_id
     * @param {number} vx
     * @param {number} vy
     */
    setVelocity(body_id, vx, vy) {
        wasm.physicsengine_setVelocity(this.__wbg_ptr, body_id, vx, vy);
    }
    /**
     * @param {number} body_id
     * @param {number} mouse_x
     * @param {number} mouse_y
     */
    startDrag(body_id, mouse_x, mouse_y) {
        wasm.physicsengine_startDrag(this.__wbg_ptr, body_id, mouse_x, mouse_y);
    }
    /**
     * @param {number} delta
     * @returns {any}
     */
    update(delta) {
        const ret = wasm.physicsengine_update(this.__wbg_ptr, delta);
        return ret;
    }
}
if (Symbol.dispose) PhysicsEngine.prototype[Symbol.dispose] = PhysicsEngine.prototype.free;

function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg_Error_83742b46f01ce22d: function(arg0, arg1) {
            const ret = Error(getStringFromWasm0(arg0, arg1));
            return ret;
        },
        __wbg___wbindgen_throw_6ddd609b62940d55: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_new_a70fbab9066b301f: function() {
            const ret = new Array();
            return ret;
        },
        __wbg_new_ab79df5bd7c26067: function() {
            const ret = new Object();
            return ret;
        },
        __wbg_set_282384002438957f: function(arg0, arg1, arg2) {
            arg0[arg1 >>> 0] = arg2;
        },
        __wbg_set_6be42768c690e380: function(arg0, arg1, arg2) {
            arg0[arg1] = arg2;
        },
        __wbindgen_cast_0000000000000001: function(arg0) {
            // Cast intrinsic for `F64 -> Externref`.
            const ret = arg0;
            return ret;
        },
        __wbindgen_cast_0000000000000002: function(arg0, arg1) {
            // Cast intrinsic for `Ref(String) -> Externref`.
            const ret = getStringFromWasm0(arg0, arg1);
            return ret;
        },
        __wbindgen_cast_0000000000000003: function(arg0) {
            // Cast intrinsic for `U64 -> Externref`.
            const ret = BigInt.asUintN(64, arg0);
            return ret;
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./matter_wasm_bg.js": import0,
    };
}

const PhysicsEngineFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_physicsengine_free(ptr >>> 0, 1));

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return decodeText(ptr, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

let wasmModule, wasm;
function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    wasmModule = module;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('matter_wasm_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
