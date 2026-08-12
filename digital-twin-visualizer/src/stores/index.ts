import { writable } from 'svelte/store';
import type { JointInfo } from '../types';

export const jointInfosStore = writable<JointInfo[]>([]);

export const selectedUpAxisStore = writable<string>('');

export const isRobotConnected = writable(false);

export const lastPointCloudData = writable(null);

export const pipeDiameterStore = writable<number>(300);
