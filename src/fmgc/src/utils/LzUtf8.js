/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

let IE10SubarrayBugPatcher; let LZUTF8; !(function (r) {
    r.runningInNodeJS = function () {
        return typeof process === 'object' && typeof process.versions === 'object' && typeof process.versions.node === 'string';
    }, r.runningInMainNodeJSModule = function () {
        return r.runningInNodeJS() && require.main === module;
    }, r.commonJSAvailable = function () {
        return typeof module === 'object' && typeof module.exports === 'object';
    }, r.runningInWebWorker = function () {
        return typeof window === 'undefined' && typeof self === 'object' && typeof self.addEventListener === 'function' && typeof self.close === 'function';
    }, r.runningInNodeChildProcess = function () {
        return r.runningInNodeJS() && typeof process.send === 'function';
    }, r.runningInNullOrigin = function () {
        return typeof window === 'object' && typeof window.location === 'object' && (document.location.protocol !== 'http:' && document.location.protocol !== 'https:');
    }, r.webWorkersAvailable = function () {
        return typeof Worker === 'function' && !r.runningInNullOrigin() && (!r.runningInNodeJS() && !(navigator && navigator.userAgent && navigator.userAgent.indexOf('Android 4.3') >= 0));
    }, r.log = function (e, t) {
        void 0 === t && (t = !1), typeof console === 'object' && (console.log(e), t && typeof document === 'object' && (document.body.innerHTML += `${e}<br/>`));
    }, r.createErrorMessage = function (e, t) {
        if (void 0 === t && (t = 'Unhandled exception'), e == null) {
            return t;
        } if (t += ': ', typeof e.content !== 'object') {
            return typeof e.content === 'string' ? t + e.content : t + e;
        } if (r.runningInNodeJS()) {
            return t + e.content.stack;
        } const n = JSON.stringify(e.content); return n !== '{}' ? t + n : t + e.content;
    }, r.printExceptionAndStackTraceToConsole = function (e, t) {
        void 0 === t && (t = 'Unhandled exception'), r.log(r.createErrorMessage(e, t));
    }, r.getGlobalObject = function () {
        return typeof global === 'object' ? global : typeof window === 'object' ? window : typeof self === 'object' ? self : {};
    }, r.toString = Object.prototype.toString, r.commonJSAvailable() && (module.exports = r);
}(LZUTF8 = LZUTF8 || {})), (function () {
    if (typeof Uint8Array === 'function' && new Uint8Array(1).subarray(1).byteLength !== 0) {
        function e(e, t) {
            function n(e, t, n) {
                return e < t ? t : n < e ? n : e;
            }e |= 0, t |= 0, arguments.length < 1 && (e = 0), arguments.length < 2 && (t = this.length), e < 0 && (e = this.length + e), t < 0 && (t = this.length + t), e = n(e, 0, this.length); let r = (t = n(t, 0, this.length)) - e; return r < 0 && (r = 0), new this.constructor(this.buffer, this.byteOffset + e * this.BYTES_PER_ELEMENT, r);
        } const t = ['Int8Array', 'Uint8Array', 'Uint8ClampedArray', 'Int16Array', 'Uint16Array', 'Int32Array', 'Uint32Array', 'Float32Array', 'Float64Array']; let n = void 0; if (typeof window === 'object' ? n = window : typeof self === 'object' && (n = self), void 0 !== n) {
            for (let r = 0; r < t.length; r++) {
                n[t[r]] && (n[t[r]].prototype.subarray = e);
            }
        }
    }
}(IE10SubarrayBugPatcher = IE10SubarrayBugPatcher || {})), (function (i) {
    let e; let u; (u = e = i.WebWorker || (i.WebWorker = {})).compressAsync = function (e, t, n) {
        let r; let o; t.inputEncoding != 'ByteArray' || e instanceof Uint8Array ? (r = {
            token: Math.random().toString(), type: 'compress', data: e, inputEncoding: t.inputEncoding, outputEncoding: t.outputEncoding,
        }, o = function (e) {
            const t = e.data; t && t.token == r.token && (u.globalWorker.removeEventListener('message', o), t.type == 'error' ? n(void 0, new Error(t.error)) : n(t.data));
        }, u.globalWorker.addEventListener('message', o), u.globalWorker.postMessage(r, [])) : n(void 0, new TypeError('compressAsync: input is not a Uint8Array'));
    }, u.decompressAsync = function (e, t, n) {
        const r = {
            token: Math.random().toString(), type: 'decompress', data: e, inputEncoding: t.inputEncoding, outputEncoding: t.outputEncoding,
        }; var o = function (e) {
            const t = e.data; t && t.token == r.token && (u.globalWorker.removeEventListener('message', o), t.type == 'error' ? n(void 0, new Error(t.error)) : n(t.data));
        }; u.globalWorker.addEventListener('message', o), u.globalWorker.postMessage(r, []);
    }, u.installWebWorkerIfNeeded = function () {
        typeof self === 'object' && void 0 === self.document && self.addEventListener != null && (self.addEventListener('message', (e) => {
            const t = e.data; if (t.type == 'compress') {
                let n = void 0; try {
                    n = i.compress(t.data, { outputEncoding: t.outputEncoding });
                } catch (e) {
                    return void self.postMessage({ token: t.token, type: 'error', error: i.createErrorMessage(e) }, []);
                }(r = {
                    token: t.token, type: 'compressionResult', data: n, encoding: t.outputEncoding,
                }).data instanceof Uint8Array && navigator.appVersion.indexOf('MSIE 10') === -1 ? self.postMessage(r, [r.data.buffer]) : self.postMessage(r, []);
            } else if (t.type == 'decompress') {
                var r; let o = void 0; try {
                    o = i.decompress(t.data, { inputEncoding: t.inputEncoding, outputEncoding: t.outputEncoding });
                } catch (e) {
                    return void self.postMessage({ token: t.token, type: 'error', error: i.createErrorMessage(e) }, []);
                }(r = {
                    token: t.token, type: 'decompressionResult', data: o, encoding: t.outputEncoding,
                }).data instanceof Uint8Array && navigator.appVersion.indexOf('MSIE 10') === -1 ? self.postMessage(r, [r.data.buffer]) : self.postMessage(r, []);
            }
        }), self.addEventListener('error', (e) => {
            i.log(i.createErrorMessage(e.error, 'Unexpected LZUTF8 WebWorker exception'));
        }));
    }, u.createGlobalWorkerIfNeeded = function () {
        return !!u.globalWorker || !!i.webWorkersAvailable() && (u.scriptURI || typeof document !== 'object' || (e = document.getElementById('lzutf8')) != null && (u.scriptURI = e.getAttribute('src') || void 0), !!u.scriptURI && (u.globalWorker = new Worker(u.scriptURI), !0)); let e;
    }, u.terminate = function () {
        u.globalWorker && (u.globalWorker.terminate(), u.globalWorker = void 0);
    }, e.installWebWorkerIfNeeded();
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    const t = (n.prototype.get = function (e) {
        return this.container[this.startPosition + e];
    }, n.prototype.getInReversedOrder = function (e) {
        return this.container[this.startPosition + this.length - 1 - e];
    }, n.prototype.set = function (e, t) {
        this.container[this.startPosition + e] = t;
    }, n); function n(e, t, n) {
        this.container = e, this.startPosition = t, this.length = n;
    }e.ArraySegment = t;
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let t; (t = e.ArrayTools || (e.ArrayTools = {})).copyElements = function (e, t, n, r, o) {
        for (;o--;) {
            n[r++] = e[t++];
        }
    }, t.zeroElements = function (e, t, n) {
        for (;n--;) {
            e[t++] = 0;
        }
    }, t.countNonzeroValuesInArray = function (e) {
        for (var t = 0, n = 0; n < e.length; n++) {
            e[n] && t++;
        } return t;
    }, t.truncateStartingElements = function (e, t) {
        if (e.length <= t) {
            throw new RangeError('truncateStartingElements: Requested length should be smaller than array length');
        } for (let n = e.length - t, r = 0; r < t; r++) {
            e[r] = e[n + r];
        }e.length = t;
    }, t.doubleByteArrayCapacity = function (e) {
        const t = new Uint8Array(2 * e.length); return t.set(e), t;
    }, t.concatUint8Arrays = function (e) {
        for (var t = 0, n = 0, r = e; n < r.length; n++) {
            t += (s = r[n]).length;
        } for (var o = new Uint8Array(t), i = 0, u = 0, a = e; u < a.length; u++) {
            var s = a[u]; o.set(s, i), i += s.length;
        } return o;
    }, t.splitByteArray = function (e, t) {
        for (var n = [], r = 0; r < e.length;) {
            const o = Math.min(t, e.length - r); n.push(e.subarray(r, r + o)), r += o;
        } return n;
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let t; (t = e.BufferTools || (e.BufferTools = {})).convertToUint8ArrayIfNeeded = function (e) {
        return typeof Buffer === 'function' && Buffer.isBuffer(e) ? t.bufferToUint8Array(e) : e;
    }, t.uint8ArrayToBuffer = function (e) {
        if (Buffer.prototype instanceof Uint8Array) {
            const t = new Uint8Array(e.buffer, e.byteOffset, e.byteLength); return Object.setPrototypeOf(t, Buffer.prototype), t;
        } for (var n = e.length, r = new Buffer(n), o = 0; o < n; o++) {
            r[o] = e[o];
        } return r;
    }, t.bufferToUint8Array = function (e) {
        if (Buffer.prototype instanceof Uint8Array) {
            return new Uint8Array(e.buffer, e.byteOffset, e.byteLength);
        } for (var t = e.length, n = new Uint8Array(t), r = 0; r < t; r++) {
            n[r] = e[r];
        } return n;
    };
}(LZUTF8 = LZUTF8 || {})), (function (o) {
    let e; (e = o.CompressionCommon || (o.CompressionCommon = {})).getCroppedBuffer = function (e, t, n, r) {
        void 0 === r && (r = 0); const o = new Uint8Array(n + r); return o.set(e.subarray(t, t + n)), o;
    }, e.getCroppedAndAppendedByteArray = function (e, t, n, r) {
        return o.ArrayTools.concatUint8Arrays([e.subarray(t, t + n), r]);
    }, e.detectCompressionSourceEncoding = function (e) {
        if (e == null) {
            throw new TypeError('detectCompressionSourceEncoding: input is null or undefined');
        } if (typeof e === 'string') {
            return 'String';
        } if (e instanceof Uint8Array || typeof Buffer === 'function' && Buffer.isBuffer(e)) {
            return 'ByteArray';
        } throw new TypeError("detectCompressionSourceEncoding: input must be of type 'string', 'Uint8Array' or 'Buffer'");
    }, e.encodeCompressedBytes = function (e, t) {
        switch (t) {
        case 'ByteArray': return e; case 'Buffer': return o.BufferTools.uint8ArrayToBuffer(e); case 'Base64': return o.encodeBase64(e); case 'BinaryString': return o.encodeBinaryString(e); case 'StorageBinaryString': return o.encodeStorageBinaryString(e); default: throw new TypeError('encodeCompressedBytes: invalid output encoding requested');
        }
    }, e.decodeCompressedBytes = function (e, t) {
        if (t == null) {
            throw new TypeError('decodeCompressedData: Input is null or undefined');
        } switch (t) {
        case 'ByteArray': case 'Buffer': var n = o.BufferTools.convertToUint8ArrayIfNeeded(e); if (!(n instanceof Uint8Array)) {
            throw new TypeError("decodeCompressedData: 'ByteArray' or 'Buffer' input type was specified but input is not a Uint8Array or Buffer");
        } return n; case 'Base64': if (typeof e !== 'string') {
            throw new TypeError("decodeCompressedData: 'Base64' input type was specified but input is not a string");
        } return o.decodeBase64(e); case 'BinaryString': if (typeof e !== 'string') {
            throw new TypeError("decodeCompressedData: 'BinaryString' input type was specified but input is not a string");
        } return o.decodeBinaryString(e); case 'StorageBinaryString': if (typeof e !== 'string') {
            throw new TypeError("decodeCompressedData: 'StorageBinaryString' input type was specified but input is not a string");
        } return o.decodeStorageBinaryString(e); default: throw new TypeError(`decodeCompressedData: invalid input encoding requested: '${t}'`);
        }
    }, e.encodeDecompressedBytes = function (e, t) {
        switch (t) {
        case 'String': return o.decodeUTF8(e); case 'ByteArray': return e; case 'Buffer': if (typeof Buffer !== 'function') {
            throw new TypeError("encodeDecompressedBytes: a 'Buffer' type was specified but is not supported at the current envirnment");
        } return o.BufferTools.uint8ArrayToBuffer(e); default: throw new TypeError('encodeDecompressedBytes: invalid output encoding requested');
        }
    };
}(LZUTF8 = LZUTF8 || {})), (function (o) {
    let t; let e; let i; let u; e = t = o.EventLoop || (o.EventLoop = {}), u = [], e.enqueueImmediate = function (e) {
        u.push(e), u.length === 1 && i();
    }, e.initializeScheduler = function () {
        function t() {
            for (let e = 0, t = u; e < t.length; e++) {
                const n = t[e]; try {
                    n.call(void 0);
                } catch (e) {
                    o.printExceptionAndStackTraceToConsole(e, 'enqueueImmediate exception');
                }
            }u.length = 0;
        } let n; let e; let r; o.runningInNodeJS() && (i = function () {
            return setImmediate(t);
        }), i = typeof window === 'object' && typeof window.addEventListener === 'function' && typeof window.postMessage === 'function' ? (n = `enqueueImmediate-${Math.random().toString()}`, window.addEventListener('message', (e) => {
            e.data === n && t();
        }), e = o.runningInNullOrigin() ? '*' : window.location.href, function () {
            return window.postMessage(n, e);
        }) : typeof MessageChannel === 'function' && typeof MessagePort === 'function' ? ((r = new MessageChannel()).port1.onmessage = t, function () {
            return r.port2.postMessage(0);
        }) : function () {
            return setTimeout(t, 0);
        };
    }, e.initializeScheduler(), o.enqueueImmediate = function (e) {
        return t.enqueueImmediate(e);
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let n; (n = e.ObjectTools || (e.ObjectTools = {})).override = function (e, t) {
        return n.extend(e, t);
    }, n.extend = function (e, t) {
        if (e == null) {
            throw new TypeError('obj is null or undefined');
        } if (typeof e !== 'object') {
            throw new TypeError('obj is not an object');
        } if (t == null && (t = {}), typeof t !== 'object') {
            throw new TypeError('newProperties is not an object');
        } if (t != null) {
            for (const n in t) {
                e[n] = t[n];
            }
        } return e;
    };
}(LZUTF8 = LZUTF8 || {})), (function (o) {
    o.getRandomIntegerInRange = function (e, t) {
        return e + Math.floor(Math.random() * (t - e));
    }, o.getRandomUTF16StringOfLength = function (e) {
        for (var t = '', n = 0; n < e; n++) {
            for (var r = void 0; (r = o.getRandomIntegerInRange(0, 1114112)) >= 55296 && r <= 57343;) {

            }t += o.Encoding.CodePoint.decodeToString(r);
        } return t;
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    const t = (n.prototype.appendCharCode = function (e) {
        this.outputBuffer[this.outputPosition++] = e, this.outputPosition === this.outputBufferCapacity && this.flushBufferToOutputString();
    }, n.prototype.appendCharCodes = function (e) {
        for (let t = 0, n = e.length; t < n; t++) {
            this.appendCharCode(e[t]);
        }
    }, n.prototype.appendString = function (e) {
        for (let t = 0, n = e.length; t < n; t++) {
            this.appendCharCode(e.charCodeAt(t));
        }
    }, n.prototype.appendCodePoint = function (e) {
        if (e <= 65535) {
            this.appendCharCode(e);
        } else {
            if (!(e <= 1114111)) {
                throw new Error(`appendCodePoint: A code point of ${e} cannot be encoded in UTF-16`);
            } this.appendCharCode(55296 + (e - 65536 >>> 10)), this.appendCharCode(56320 + (e - 65536 & 1023));
        }
    }, n.prototype.getOutputString = function () {
        return this.flushBufferToOutputString(), this.outputString;
    }, n.prototype.flushBufferToOutputString = function () {
        this.outputPosition === this.outputBufferCapacity ? this.outputString += String.fromCharCode.apply(null, this.outputBuffer) : this.outputString += String.fromCharCode.apply(null, this.outputBuffer.subarray(0, this.outputPosition)), this.outputPosition = 0;
    }, n); function n(e) {
        void 0 === e && (e = 1024), this.outputBufferCapacity = e, this.outputPosition = 0, this.outputString = '', this.outputBuffer = new Uint16Array(this.outputBufferCapacity);
    }e.StringBuilder = t;
}(LZUTF8 = LZUTF8 || {})), (function (o) {
    const e = (t.prototype.restart = function () {
        this.startTime = t.getTimestamp();
    }, t.prototype.getElapsedTime = function () {
        return t.getTimestamp() - this.startTime;
    }, t.prototype.getElapsedTimeAndRestart = function () {
        const e = this.getElapsedTime(); return this.restart(), e;
    }, t.prototype.logAndRestart = function (e, t) {
        void 0 === t && (t = !0); const n = this.getElapsedTime(); const r = `${e}: ${n.toFixed(3)}ms`; return o.log(r, t), this.restart(), n;
    }, t.getTimestamp = function () {
        return this.timestampFunc || this.createGlobalTimestampFunction(), this.timestampFunc();
    }, t.getMicrosecondTimestamp = function () {
        return Math.floor(1e3 * t.getTimestamp());
    }, t.createGlobalTimestampFunction = function () {
        let n; let e; let t; let r; typeof process === 'object' && typeof process.hrtime === 'function' ? (n = 0, this.timestampFunc = function () {
            const e = process.hrtime(); const t = 1e3 * e[0] + e[1] / 1e6; return n + t;
        }, n = Date.now() - this.timestampFunc()) : typeof chrome === 'object' && chrome.Interval ? (e = Date.now(), (t = new chrome.Interval()).start(), this.timestampFunc = function () {
            return e + t.microseconds() / 1e3;
        }) : typeof performance === 'object' && performance.now ? (r = Date.now() - performance.now(), this.timestampFunc = function () {
            return r + performance.now();
        }) : Date.now ? this.timestampFunc = function () {
            return Date.now();
        } : this.timestampFunc = function () {
            return (new Date()).getTime();
        };
    }, t); function t() {
        this.restart();
    }o.Timer = e;
}(LZUTF8 = LZUTF8 || {})), (function (r) {
    const e = (t.prototype.compressBlock = function (e) {
        if (e == null) {
            throw new TypeError('compressBlock: undefined or null input received');
        } return typeof e === 'string' && (e = r.encodeUTF8(e)), e = r.BufferTools.convertToUint8ArrayIfNeeded(e), this.compressUtf8Block(e);
    }, t.prototype.compressUtf8Block = function (e) {
        if (!e || e.length == 0) {
            return new Uint8Array(0);
        } const t = this.cropAndAddNewBytesToInputBuffer(e); const n = this.inputBuffer; const r = this.inputBuffer.length; this.outputBuffer = new Uint8Array(e.length); for (let o = this.outputBufferPosition = 0, i = t; i < r; i++) {
            var u; var a; var s; const c = n[i]; let f = i < o; i > r - this.MinimumSequenceLength ? f || this.outputRawByte(c) : (u = this.getBucketIndexForPrefix(i), f || (a = this.findLongestMatch(i, u)) != null && (this.outputPointerBytes(a.length, a.distance), o = i + a.length, f = !0), f || this.outputRawByte(c), s = this.inputBufferStreamOffset + i, this.prefixHashTable.addValueToBucket(u, s));
        } return this.outputBuffer.subarray(0, this.outputBufferPosition);
    }, t.prototype.findLongestMatch = function (e, t) {
        const n = this.prefixHashTable.getArraySegmentForBucketIndex(t, this.reusableArraySegmentObject); if (n == null) {
            return null;
        } for (var r, o = this.inputBuffer, i = 0, u = 0; u < n.length; u++) {
            const a = n.getInReversedOrder(u) - this.inputBufferStreamOffset; const s = e - a; var c = void 0; var c = void 0 === r ? this.MinimumSequenceLength - 1 : r < 128 && s >= 128 ? i + (i >>> 1) : i; if (s > this.MaximumMatchDistance || c >= this.MaximumSequenceLength || e + c >= o.length) {
                break;
            } if (o[a + c] === o[e + c]) {
                for (let f = 0; ;f++) {
                    if (e + f === o.length || o[a + f] !== o[e + f]) {
                        c < f && (r = s, i = f); break;
                    } if (f === this.MaximumSequenceLength) {
                        return { distance: s, length: this.MaximumSequenceLength };
                    }
                }
            }
        } return void 0 !== r ? { distance: r, length: i } : null;
    }, t.prototype.getBucketIndexForPrefix = function (e) {
        return (7880599 * this.inputBuffer[e] + 39601 * this.inputBuffer[e + 1] + 199 * this.inputBuffer[e + 2] + this.inputBuffer[e + 3]) % this.PrefixHashTableSize;
    }, t.prototype.outputPointerBytes = function (e, t) {
        t < 128 ? (this.outputRawByte(192 | e), this.outputRawByte(t)) : (this.outputRawByte(224 | e), this.outputRawByte(t >>> 8), this.outputRawByte(255 & t));
    }, t.prototype.outputRawByte = function (e) {
        this.outputBuffer[this.outputBufferPosition++] = e;
    }, t.prototype.cropAndAddNewBytesToInputBuffer = function (e) {
        if (void 0 === this.inputBuffer) {
            return this.inputBuffer = e, 0;
        } const t = Math.min(this.inputBuffer.length, this.MaximumMatchDistance); const n = this.inputBuffer.length - t; return this.inputBuffer = r.CompressionCommon.getCroppedAndAppendedByteArray(this.inputBuffer, n, t, e), this.inputBufferStreamOffset += n, t;
    }, t); function t(e) {
        void 0 === e && (e = !0), this.MinimumSequenceLength = 4, this.MaximumSequenceLength = 31, this.MaximumMatchDistance = 32767, this.PrefixHashTableSize = 65537, this.inputBufferStreamOffset = 1, e && typeof Uint32Array === 'function' ? this.prefixHashTable = new r.CompressorCustomHashTable(this.PrefixHashTableSize) : this.prefixHashTable = new r.CompressorSimpleHashTable(this.PrefixHashTableSize);
    }r.Compressor = e;
}(LZUTF8 = LZUTF8 || {})), (function (a) {
    const e = (t.prototype.addValueToBucket = function (e, t) {
        e <<= 1, this.storageIndex >= this.storage.length >>> 1 && this.compact(); let n; let r; let o = this.bucketLocators[e]; o === 0 ? (o = this.storageIndex, n = 1, this.storage[this.storageIndex] = t, this.storageIndex += this.minimumBucketCapacity) : ((n = this.bucketLocators[e + 1]) === this.maximumBucketCapacity - 1 && (n = this.truncateBucketToNewerElements(o, n, this.maximumBucketCapacity / 2)), r = o + n, this.storage[r] === 0 ? (this.storage[r] = t, r === this.storageIndex && (this.storageIndex += n)) : (a.ArrayTools.copyElements(this.storage, o, this.storage, this.storageIndex, n), o = this.storageIndex, this.storageIndex += n, this.storage[this.storageIndex++] = t, this.storageIndex += n), n++), this.bucketLocators[e] = o, this.bucketLocators[e + 1] = n;
    }, t.prototype.truncateBucketToNewerElements = function (e, t, n) {
        const r = e + t - n; return a.ArrayTools.copyElements(this.storage, r, this.storage, e, n), a.ArrayTools.zeroElements(this.storage, e + n, t - n), n;
    }, t.prototype.compact = function () {
        const e = this.bucketLocators; const t = this.storage; this.bucketLocators = new Uint32Array(this.bucketLocators.length), this.storageIndex = 1; for (var n = 0; n < e.length; n += 2) {
            const r = e[n + 1]; r !== 0 && (this.bucketLocators[n] = this.storageIndex, this.bucketLocators[n + 1] = r, this.storageIndex += Math.max(Math.min(2 * r, this.maximumBucketCapacity), this.minimumBucketCapacity));
        } for (this.storage = new Uint32Array(8 * this.storageIndex), n = 0; n < e.length; n += 2) {
            var o; var i; const u = e[n]; u !== 0 && (o = this.bucketLocators[n], i = this.bucketLocators[n + 1], a.ArrayTools.copyElements(t, u, this.storage, o, i));
        }
    }, t.prototype.getArraySegmentForBucketIndex = function (e, t) {
        e <<= 1; const n = this.bucketLocators[e]; return n === 0 ? null : (void 0 === t && (t = new a.ArraySegment(this.storage, n, this.bucketLocators[e + 1])), t);
    }, t.prototype.getUsedBucketCount = function () {
        return Math.floor(a.ArrayTools.countNonzeroValuesInArray(this.bucketLocators) / 2);
    }, t.prototype.getTotalElementCount = function () {
        for (var e = 0, t = 0; t < this.bucketLocators.length; t += 2) {
            e += this.bucketLocators[t + 1];
        } return e;
    }, t); function t(e) {
        this.minimumBucketCapacity = 4, this.maximumBucketCapacity = 64, this.bucketLocators = new Uint32Array(2 * e), this.storage = new Uint32Array(2 * e), this.storageIndex = 1;
    }a.CompressorCustomHashTable = e;
}(LZUTF8 = LZUTF8 || {})), (function (r) {
    const e = (t.prototype.addValueToBucket = function (e, t) {
        const n = this.buckets[e]; void 0 === n ? this.buckets[e] = [t] : (n.length === this.maximumBucketCapacity - 1 && r.ArrayTools.truncateStartingElements(n, this.maximumBucketCapacity / 2), n.push(t));
    }, t.prototype.getArraySegmentForBucketIndex = function (e, t) {
        const n = this.buckets[e]; return void 0 === n ? null : (void 0 === t && (t = new r.ArraySegment(n, 0, n.length)), t);
    }, t.prototype.getUsedBucketCount = function () {
        return r.ArrayTools.countNonzeroValuesInArray(this.buckets);
    }, t.prototype.getTotalElementCount = function () {
        for (var e = 0, t = 0; t < this.buckets.length; t++) {
            void 0 !== this.buckets[t] && (e += this.buckets[t].length);
        } return e;
    }, t); function t(e) {
        this.maximumBucketCapacity = 64, this.buckets = new Array(e);
    }r.CompressorSimpleHashTable = e;
}(LZUTF8 = LZUTF8 || {})), (function (f) {
    const e = (t.prototype.decompressBlockToString = function (e) {
        return e = f.BufferTools.convertToUint8ArrayIfNeeded(e), f.decodeUTF8(this.decompressBlock(e));
    }, t.prototype.decompressBlock = function (e) {
        this.inputBufferRemainder && (e = f.ArrayTools.concatUint8Arrays([this.inputBufferRemainder, e]), this.inputBufferRemainder = void 0); for (var t = this.cropOutputBufferToWindowAndInitialize(Math.max(4 * e.length, 1024)), n = 0, r = e.length; n < r; n++) {
            const o = e[n]; if (o >>> 6 == 3) {
                const i = o >>> 5; if (n == r - 1 || n == r - 2 && i == 7) {
                    this.inputBufferRemainder = e.subarray(n); break;
                } if (e[n + 1] >>> 7 == 1) {
                    this.outputByte(o);
                } else {
                    const u = 31 & o; let a = void 0; i == 6 ? (a = e[n + 1], n += 1) : (a = e[n + 1] << 8 | e[n + 2], n += 2); for (let s = this.outputPosition - a, c = 0; c < u; c++) {
                        this.outputByte(this.outputBuffer[s + c]);
                    }
                }
            } else {
                this.outputByte(o);
            }
        } return this.rollBackIfOutputBufferEndsWithATruncatedMultibyteSequence(), f.CompressionCommon.getCroppedBuffer(this.outputBuffer, t, this.outputPosition - t);
    }, t.prototype.outputByte = function (e) {
        this.outputPosition === this.outputBuffer.length && (this.outputBuffer = f.ArrayTools.doubleByteArrayCapacity(this.outputBuffer)), this.outputBuffer[this.outputPosition++] = e;
    }, t.prototype.cropOutputBufferToWindowAndInitialize = function (e) {
        if (!this.outputBuffer) {
            return this.outputBuffer = new Uint8Array(e), 0;
        } const t = Math.min(this.outputPosition, this.MaximumMatchDistance); if (this.outputBuffer = f.CompressionCommon.getCroppedBuffer(this.outputBuffer, this.outputPosition - t, t, e), this.outputPosition = t, this.outputBufferRemainder) {
            for (let n = 0; n < this.outputBufferRemainder.length; n++) {
                this.outputByte(this.outputBufferRemainder[n]);
            } this.outputBufferRemainder = void 0;
        } return t;
    }, t.prototype.rollBackIfOutputBufferEndsWithATruncatedMultibyteSequence = function () {
        for (let e = 1; e <= 4 && this.outputPosition - e >= 0; e++) {
            const t = this.outputBuffer[this.outputPosition - e]; if (e < 4 && t >>> 3 == 30 || e < 3 && t >>> 4 == 14 || e < 2 && t >>> 5 == 6) {
                return this.outputBufferRemainder = this.outputBuffer.subarray(this.outputPosition - e, this.outputPosition), void (this.outputPosition -= e);
            }
        }
    }, t); function t() {
        this.MaximumMatchDistance = 32767, this.outputPosition = 0;
    }f.Decompressor = e;
}(LZUTF8 = LZUTF8 || {})), (function (a) {
    let e; let t; let s; let c; e = a.Encoding || (a.Encoding = {}), t = e.Base64 || (e.Base64 = {}), s = new Uint8Array([65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 43, 47]), c = new Uint8Array([255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 62, 255, 255, 255, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 255, 255, 255, 0, 255, 255, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 255, 255, 255, 255, 255, 255, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 255, 255, 255, 255]), t.encode = function (e) {
        return e && e.length != 0 ? a.runningInNodeJS() ? a.BufferTools.uint8ArrayToBuffer(e).toString('base64') : t.encodeWithJS(e) : '';
    }, t.decode = function (e) {
        return e ? a.runningInNodeJS() ? a.BufferTools.bufferToUint8Array(new Buffer(e, 'base64')) : t.decodeWithJS(e) : new Uint8Array(0);
    }, t.encodeWithJS = function (e, t) {
        if (void 0 === t && (t = !0), !e || e.length == 0) {
            return '';
        } for (var n, r = s, o = new a.StringBuilder(), i = 0, u = e.length; i < u; i += 3) {
            i <= u - 3 ? (n = e[i] << 16 | e[i + 1] << 8 | e[i + 2], o.appendCharCode(r[n >>> 18 & 63]), o.appendCharCode(r[n >>> 12 & 63]), o.appendCharCode(r[n >>> 6 & 63]), o.appendCharCode(r[63 & n]), n = 0) : i === u - 2 ? (n = e[i] << 16 | e[i + 1] << 8, o.appendCharCode(r[n >>> 18 & 63]), o.appendCharCode(r[n >>> 12 & 63]), o.appendCharCode(r[n >>> 6 & 63]), t && o.appendCharCode(61)) : i === u - 1 && (n = e[i] << 16, o.appendCharCode(r[n >>> 18 & 63]), o.appendCharCode(r[n >>> 12 & 63]), t && (o.appendCharCode(61), o.appendCharCode(61)));
        } return o.getOutputString();
    }, t.decodeWithJS = function (e, t) {
        if (!e || e.length == 0) {
            return new Uint8Array(0);
        } const n = e.length % 4; if (n == 1) {
            throw new Error('Invalid Base64 string: length % 4 == 1');
        } n == 2 ? e += '==' : n == 3 && (e += '='), t = t || new Uint8Array(e.length); for (var r = 0, o = e.length, i = 0; i < o; i += 4) {
            const u = c[e.charCodeAt(i)] << 18 | c[e.charCodeAt(i + 1)] << 12 | c[e.charCodeAt(i + 2)] << 6 | c[e.charCodeAt(i + 3)]; t[r++] = u >>> 16 & 255, t[r++] = u >>> 8 & 255, t[r++] = 255 & u;
        } return e.charCodeAt(o - 1) == 61 && r--, e.charCodeAt(o - 2) == 61 && r--, t.subarray(0, r);
    };
}(LZUTF8 = LZUTF8 || {})), (function (a) {
    let e; let t; e = a.Encoding || (a.Encoding = {}), (t = e.BinaryString || (e.BinaryString = {})).encode = function (e) {
        if (e == null) {
            throw new TypeError('BinaryString.encode: undefined or null input received');
        } if (e.length === 0) {
            return '';
        } for (var t = e.length, n = new a.StringBuilder(), r = 0, o = 1, i = 0; i < t; i += 2) {
            var u = void 0; var u = i == t - 1 ? e[i] << 8 : e[i] << 8 | e[i + 1]; n.appendCharCode(r << 16 - o | u >>> o), r = u & (1 << o) - 1, o === 15 ? (n.appendCharCode(r), r = 0, o = 1) : o += 1, t - 2 <= i && n.appendCharCode(r << 16 - o);
        } return n.appendCharCode(32768 | t % 2), n.getOutputString();
    }, t.decode = function (e) {
        if (typeof e !== 'string') {
            throw new TypeError('BinaryString.decode: invalid input type');
        } if (e == '') {
            return new Uint8Array(0);
        } for (var t, n = new Uint8Array(3 * e.length), r = 0, o = 0, i = 0, u = 0; u < e.length; u++) {
            const a = e.charCodeAt(u); a >= 32768 ? (a == 32769 && r--, i = 0) : (o = i == 0 ? a : (t = o << i | a >>> 15 - i, n[r++] = t >>> 8, n[r++] = 255 & t, a & (1 << 15 - i) - 1), i == 15 ? i = 0 : i += 1);
        } return n.subarray(0, r);
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let t; let n; t = e.Encoding || (e.Encoding = {}), (n = t.CodePoint || (t.CodePoint = {})).encodeFromString = function (e, t) {
        const n = e.charCodeAt(t); if (n < 55296 || n > 56319) {
            return n;
        } const r = e.charCodeAt(t + 1); if (r >= 56320 && r <= 57343) {
            return r - 56320 + (n - 55296 << 10) + 65536;
        } throw new Error(`getUnicodeCodePoint: Received a lead surrogate character, char code ${n}, followed by ${r}, which is not a trailing surrogate character code.`);
    }, n.decodeToString = function (e) {
        if (e <= 65535) {
            return String.fromCharCode(e);
        } if (e <= 1114111) {
            return String.fromCharCode(55296 + (e - 65536 >>> 10), 56320 + (e - 65536 & 1023));
        } throw new Error(`getStringFromUnicodeCodePoint: A code point of ${e} cannot be encoded in UTF-16`);
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let t; let n; let r; t = e.Encoding || (e.Encoding = {}), n = t.DecimalString || (t.DecimalString = {}), r = ['000', '001', '002', '003', '004', '005', '006', '007', '008', '009', '010', '011', '012', '013', '014', '015', '016', '017', '018', '019', '020', '021', '022', '023', '024', '025', '026', '027', '028', '029', '030', '031', '032', '033', '034', '035', '036', '037', '038', '039', '040', '041', '042', '043', '044', '045', '046', '047', '048', '049', '050', '051', '052', '053', '054', '055', '056', '057', '058', '059', '060', '061', '062', '063', '064', '065', '066', '067', '068', '069', '070', '071', '072', '073', '074', '075', '076', '077', '078', '079', '080', '081', '082', '083', '084', '085', '086', '087', '088', '089', '090', '091', '092', '093', '094', '095', '096', '097', '098', '099', '100', '101', '102', '103', '104', '105', '106', '107', '108', '109', '110', '111', '112', '113', '114', '115', '116', '117', '118', '119', '120', '121', '122', '123', '124', '125', '126', '127', '128', '129', '130', '131', '132', '133', '134', '135', '136', '137', '138', '139', '140', '141', '142', '143', '144', '145', '146', '147', '148', '149', '150', '151', '152', '153', '154', '155', '156', '157', '158', '159', '160', '161', '162', '163', '164', '165', '166', '167', '168', '169', '170', '171', '172', '173', '174', '175', '176', '177', '178', '179', '180', '181', '182', '183', '184', '185', '186', '187', '188', '189', '190', '191', '192', '193', '194', '195', '196', '197', '198', '199', '200', '201', '202', '203', '204', '205', '206', '207', '208', '209', '210', '211', '212', '213', '214', '215', '216', '217', '218', '219', '220', '221', '222', '223', '224', '225', '226', '227', '228', '229', '230', '231', '232', '233', '234', '235', '236', '237', '238', '239', '240', '241', '242', '243', '244', '245', '246', '247', '248', '249', '250', '251', '252', '253', '254', '255'], n.encode = function (e) {
        for (var t = [], n = 0; n < e.length; n++) {
            t.push(r[e[n]]);
        } return t.join(' ');
    };
}(LZUTF8 = LZUTF8 || {})), (function (e) {
    let t; let n; t = e.Encoding || (e.Encoding = {}), (n = t.StorageBinaryString || (t.StorageBinaryString = {})).encode = function (e) {
        return t.BinaryString.encode(e).replace(/\0/g, '耂');
    }, n.decode = function (e) {
        return t.BinaryString.decode(e.replace(/\u8002/g, '\0'));
    };
}(LZUTF8 = LZUTF8 || {})), (function (s) {
    let i; let t; let n; let r; i = s.Encoding || (s.Encoding = {}), (t = i.UTF8 || (i.UTF8 = {})).encode = function (e) {
        return e && e.length != 0 ? s.runningInNodeJS() ? s.BufferTools.bufferToUint8Array(new Buffer(e, 'utf8')) : t.createNativeTextEncoderAndDecoderIfAvailable() ? n.encode(e) : t.encodeWithJS(e) : new Uint8Array(0);
    }, t.decode = function (e) {
        return e && e.length != 0 ? s.runningInNodeJS() ? s.BufferTools.uint8ArrayToBuffer(e).toString('utf8') : t.createNativeTextEncoderAndDecoderIfAvailable() ? r.decode(e) : t.decodeWithJS(e) : '';
    }, t.encodeWithJS = function (e, t) {
        if (!e || e.length == 0) {
            return new Uint8Array(0);
        } t = t || new Uint8Array(4 * e.length); for (var n = 0, r = 0; r < e.length; r++) {
            const o = i.CodePoint.encodeFromString(e, r); if (o <= 127) {
                t[n++] = o;
            } else if (o <= 2047) {
                t[n++] = 192 | o >>> 6, t[n++] = 128 | 63 & o;
            } else if (o <= 65535) {
                t[n++] = 224 | o >>> 12, t[n++] = 128 | o >>> 6 & 63, t[n++] = 128 | 63 & o;
            } else {
                if (!(o <= 1114111)) {
                    throw new Error('Invalid UTF-16 string: Encountered a character unsupported by UTF-8/16 (RFC 3629)');
                } t[n++] = 240 | o >>> 18, t[n++] = 128 | o >>> 12 & 63, t[n++] = 128 | o >>> 6 & 63, t[n++] = 128 | 63 & o, r++;
            }
        } return t.subarray(0, n);
    }, t.decodeWithJS = function (e, t, n) {
        if (void 0 === t && (t = 0), !e || e.length == 0) {
            return '';
        } void 0 === n && (n = e.length); for (var r, o, i = new s.StringBuilder(), u = t, a = n; u < a;) {
            if ((o = e[u]) >>> 7 == 0) {
                r = o, u += 1;
            } else if (o >>> 5 == 6) {
                if (n <= u + 1) {
                    throw new Error(`Invalid UTF-8 stream: Truncated codepoint sequence encountered at position ${u}`);
                } r = (31 & o) << 6 | 63 & e[u + 1], u += 2;
            } else if (o >>> 4 == 14) {
                if (n <= u + 2) {
                    throw new Error(`Invalid UTF-8 stream: Truncated codepoint sequence encountered at position ${u}`);
                } r = (15 & o) << 12 | (63 & e[u + 1]) << 6 | 63 & e[u + 2], u += 3;
            } else {
                if (o >>> 3 != 30) {
                    throw new Error(`Invalid UTF-8 stream: An invalid lead byte value encountered at position ${u}`);
                } if (n <= u + 3) {
                    throw new Error(`Invalid UTF-8 stream: Truncated codepoint sequence encountered at position ${u}`);
                } r = (7 & o) << 18 | (63 & e[u + 1]) << 12 | (63 & e[u + 2]) << 6 | 63 & e[u + 3], u += 4;
            }i.appendCodePoint(r);
        } return i.getOutputString();
    }, t.createNativeTextEncoderAndDecoderIfAvailable = function () {
        return !!n || typeof TextEncoder === 'function' && (n = new TextEncoder('utf-8'), r = new TextDecoder('utf-8'), !0);
    };
}(LZUTF8 = LZUTF8 || {})), (function (o) {
    o.compress = function (e, t) {
        if (void 0 === t && (t = {}), e == null) {
            throw new TypeError('compress: undefined or null input received');
        } const n = o.CompressionCommon.detectCompressionSourceEncoding(e); t = o.ObjectTools.override({ inputEncoding: n, outputEncoding: 'ByteArray' }, t); const r = (new o.Compressor()).compressBlock(e); return o.CompressionCommon.encodeCompressedBytes(r, t.outputEncoding);
    }, o.decompress = function (e, t) {
        if (void 0 === t && (t = {}), e == null) {
            throw new TypeError('decompress: undefined or null input received');
        } t = o.ObjectTools.override({ inputEncoding: 'ByteArray', outputEncoding: 'String' }, t); const n = o.CompressionCommon.decodeCompressedBytes(e, t.inputEncoding); const r = (new o.Decompressor()).decompressBlock(n); return o.CompressionCommon.encodeDecompressedBytes(r, t.outputEncoding);
    }, o.compressAsync = function (e, t, n) {
        let r; n == null && (n = function () {}); try {
            r = o.CompressionCommon.detectCompressionSourceEncoding(e);
        } catch (e) {
            return void n(void 0, e);
        }t = o.ObjectTools.override({
            inputEncoding: r, outputEncoding: 'ByteArray', useWebWorker: !0, blockSize: 65536,
        }, t), o.enqueueImmediate(() => {
            t.useWebWorker && o.WebWorker.createGlobalWorkerIfNeeded() ? o.WebWorker.compressAsync(e, t, n) : o.AsyncCompressor.compressAsync(e, t, n);
        });
    }, o.decompressAsync = function (e, t, n) {
        let r; n == null && (n = function () {}), e != null ? (t = o.ObjectTools.override({
            inputEncoding: 'ByteArray', outputEncoding: 'String', useWebWorker: !0, blockSize: 65536,
        }, t), r = o.BufferTools.convertToUint8ArrayIfNeeded(e), o.EventLoop.enqueueImmediate(() => {
            t.useWebWorker && o.WebWorker.createGlobalWorkerIfNeeded() ? o.WebWorker.decompressAsync(r, t, n) : o.AsyncDecompressor.decompressAsync(e, t, n);
        })) : n(void 0, new TypeError('decompressAsync: undefined or null input received'));
    }, o.createCompressionStream = function () {
        return o.AsyncCompressor.createCompressionStream();
    }, o.createDecompressionStream = function () {
        return o.AsyncDecompressor.createDecompressionStream();
    }, o.encodeUTF8 = function (e) {
        return o.Encoding.UTF8.encode(e);
    }, o.decodeUTF8 = function (e) {
        return o.Encoding.UTF8.decode(e);
    }, o.encodeBase64 = function (e) {
        return o.Encoding.Base64.encode(e);
    }, o.decodeBase64 = function (e) {
        return o.Encoding.Base64.decode(e);
    }, o.encodeBinaryString = function (e) {
        return o.Encoding.BinaryString.encode(e);
    }, o.decodeBinaryString = function (e) {
        return o.Encoding.BinaryString.decode(e);
    }, o.encodeStorageBinaryString = function (e) {
        return o.Encoding.StorageBinaryString.encode(e);
    }, o.decodeStorageBinaryString = function (e) {
        return o.Encoding.StorageBinaryString.decode(e);
    };
}(LZUTF8 = LZUTF8 || {}));
