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

'use strict';

const babel = require('@rollup/plugin-babel').default;
const commonjs = require('@rollup/plugin-commonjs');
const nodeResolve = require('@rollup/plugin-node-resolve').default;
const replace = require('@rollup/plugin-replace');

const extensions = ['.js', '.ts'];

module.exports = {
    input: `${__dirname}/src/wtsdk.ts`,
    plugins: [
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
        file: `${__dirname}/../../A32NX/html_ui/JS/flightPlan/bundle.js`,
        globals: {
            WorkingTitle: 'WorkingTitle',
        },
        format: 'umd',
        name: 'fpm',
    },
};
