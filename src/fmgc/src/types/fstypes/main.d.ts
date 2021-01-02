/*
 * A32NX
 * Copyright (C) 2020-2021 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// <reference path='./FSTypes.d.ts' />

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