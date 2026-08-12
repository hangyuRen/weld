<script lang="ts">
  import { jointInfosStore, selectedUpAxisStore } from "../../stores";
  import type { JointInfo } from "../../types";
  import Interface from "../Interface/InterfaceDrawer.svelte";
  import Scene from "../Scene/index.svelte";
  import Monitor from "../Monitor/Monitor.svelte";
  import PointCloudPanel from '../PointCloud/PointCloudPanel.svelte';
  import WSClient from '../../wsClient/wsClient';
  import { isRobotConnected } from '../../stores';
  import CurrentWSClient from "../../wsClient/CurrentWSClient";

  export let chartA: any = null;
  export let chartB: any = null;

  // 控制界面显示的变量
  let activeTab: 'robot' | 'monitor' = 'monitor';

    // 点云面板折叠状态
  let leftCollapsed = false;
  let rightCollapsed = false;

  let jointInfos: JointInfo[] = [];
  jointInfosStore.subscribe((value: JointInfo[]): void => {
    jointInfos = value;
  });

  let selectedUpAxis = "";
  selectedUpAxisStore.subscribe((value: string): void => {
    selectedUpAxis = value;
  });

  // 切换函数
  const toggleView = () => {
    activeTab = activeTab === 'robot' ? 'monitor' : 'robot';
  };

  let wsClient;
  let currentWSClient;

  // 只有当 store 变为 true 时才初始化并启动
  $: if ($isRobotConnected) {
    if (!wsClient) {
      wsClient = new WSClient('ws://127.0.0.1:8080');
      wsClient.start();
      console.log("关节角度 WebSocket 已启动");
    }

    if (!currentWSClient) {
      currentWSClient = new CurrentWSClient('ws://127.0.0.1:8081', (armA, armB) => {
        if (chartA) chartA.updateData(armA);
        if (chartB) chartB.updateData(armB);
      });
      currentWSClient.start()
    }
  }
</script>

<!--
  When passing `props` to a child component, use `bind` only if it is
  a two way binding, which means the child component will also update the `props`
  in the parent component.
-->
<div class="fixed top-4 left-4 z-[100]">
  <button class="btn btn-primary shadow-lg" on:click={toggleView}>
    {#if activeTab === 'robot'}
      进入监控看板 →
    {:else}
      ← 返回 3D 模型
    {/if}
  </button>
</div>

<div class="relative w-full h-screen overflow-hidden bg-base-100">
  {#if activeTab === 'robot'}
    <div class="drawer h-full">
      <Interface bind:jointInfos bind:selectedUpAxis />
      <Scene {jointInfos} {selectedUpAxis} />
         <!-- 左侧点云展示框 -->
<!--    <div class="pc-slot pc-left" class:collapsed={leftCollapsed}>-->
<!--      <PointCloudPanel title="左侧管道点云" bind:collapsed={leftCollapsed} />-->
<!--    </div>-->

<!--    &lt;!&ndash; 右侧点云展示框 &ndash;&gt;-->
<!--    <div class="pc-slot pc-right" class:collapsed={rightCollapsed}>-->
<!--      <PointCloudPanel title="右侧管道点云" bind:collapsed={rightCollapsed} />-->
<!--    </div>-->
    </div>
  {:else}
    <Monitor bind:chartA bind:chartB/>
  {/if}
</div>

<style global lang="postcss">
  @tailwind base;
  @tailwind components;
  @tailwind utilities;

  /* 彻底禁掉 body 的滚动条，防止布局溢出 */
  :global(html, body) {
    margin: 0;
    padding: 0;
    width: 100vw;
    height: 100vh;
    overflow: hidden;
  }

  /* 确保主容器可被绝对定位叠加 */
  .app-root {
    position: relative;
    width: 100%;
    height: 100%;
  }

  .pc-slot {
    position: absolute;
    z-index: 50; /* 保证盖在three主场景上 */
    pointer-events: auto;
  }

  /* 左框：对应你截图左上红框 */
  .pc-left {
    top: 14px;
    left: 14px;
    width: min(38vw, 520px);
    /* 拉长点云展示区域（红框下半部分会跟随变大），同时不与上方控件区重叠 */
    height: min(52vh, 520px);
    min-width: 320px;
    min-height: 240px;
  }

  /* 右框：对应你截图右上红框（避开右上“进入沉浸模式”按钮） */
  .pc-right {
    top: 78px;
    right: 14px;
    width: min(30vw, 420px);
    height: min(46vh, 460px);
    min-width: 280px;
    min-height: 220px;
  }
/* 小屏适配：避免挡住主体 */
  @media (max-width: 980px) {
    .pc-left {
      width: 360px;
      height: 360px;
    }
    .pc-right {
      top: 360px;
      width: 360px;
      height: 360px;
    }
  }

  /* 折叠时只保留标题栏高度，避免占用遮挡 */
  .pc-slot.collapsed {
    height: 34px !important;
    min-height: 34px !important;
  }
</style>
