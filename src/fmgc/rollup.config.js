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

'use strict';

const babel = require('@rollup/plugin-babel').default;
const commonjs = require('@rollup/plugin-commonjs');
const nodeResolve = require('@rollup/plugin-node-resolve').default;
const replace = require('@rollup/plugin-replace');
const copy = require('rollup-plugin-copy');

const extensions = ['.js', '.ts'];

module.exports = {
    input: `${__dirname}/src/wtsdk.ts`,
    plugins: [
        copy({
            targets: [
                {
                    src: `${__dirname}/src/utils/LzUtf8.js`,
                    dest: `${__dirname}/../../A32NX/html_ui/JS/fmgc/`,
                },
            ],
        }),
        replace({
            'process.env.NODE_ENV': JSON.stringify('production'),
        }),
        commonjs(),
        babel({
            presets: ['@babel/preset-typescript', ['@babel/preset-env', {
                targets: { browsers: ['safari 11'] },
            }]],
            plugins: [
                '@babel/plugin-proposal-class-properties',
            ],
            extensions,
        }),
        nodeResolve({
            extensions,
        }),
    ],
    external: ['MSFS', 'WorkingTitle'],
    output: {
        file: `${__dirname}/../../A32NX/html_ui/JS/fmgc/bundle.js`,
        globals: {
            WorkingTitle: 'WorkingTitle',
        },
        format: 'umd',
        name: 'fpm',
    },
};
