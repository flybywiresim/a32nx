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

/// <reference path='./FSTypes.d.ts' />

declare const LZUTF8: any;

declare module "WorkingTitle" {
  export class WTDataStore {
    /**
     * Retrieves a key from the datastore, possibly returning the default value
     * @param key The name of the key to retrieve
     * @param defaultValue The default value to use if the key does not exist
     * @returns Either the stored value of the key, or the default value
     */
    static get(key: string, defaultValue: string | number | boolean): any;

    /**
     * Stores a key in the datastore
     * @param key The name of the value to store
     * @param The value to store
     */
    static set(key: string, value: string | number | boolean): any;
  }

  export class LZUTF8 {
    static compress(input: string, options?: {}): any;
    static decompress(input: string, options?: {}): any;
  }
}