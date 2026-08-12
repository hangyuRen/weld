<!-- InterfaceDrawer.svelte -->
<script lang="ts">
    import Interface from "../Interface/index.svelte";
    import type { JointInfo } from "../../types";

    export let jointInfos: JointInfo[] = [];
    export let selectedUpAxis: string;

    export let open: boolean = false;

    // 调整这里的宽度，单位 rem（1rem 通常等于16px）
    const drawerWidthRem = 29;

    function toggle() {
        open = !open;
    }

    function close() {
        open = false;
    }
</script>

<div class="relative">
    {#if open}
        <!-- 半透明遮罩，点击关闭 -->
        <div
                class="fixed inset-0 bg-black/30 z-30"
                on:click={close}
                aria-hidden="true"
        />
    {/if}

    <!-- toggle 按钮 -->
    <button
            class="fixed top-1/2 z-40 -translate-y-1/2 p-2 rounded-r-md shadow-md bg-white/90 hover:bg-white focus:outline-none focus:ring"
            aria-expanded={open}
            on:click={toggle}
            style="left: {open ? `${drawerWidthRem}rem` : '0.5rem'}; transition:left 300ms ease;"
            title={open ? '关闭控制面板' : '打开控制面板'}
    >
        <svg
                xmlns="http://www.w3.org/2000/svg"
                class="w-6 h-6 transform transition-transform duration-300"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                stroke-width="2"
                stroke-linecap="round"
                stroke-linejoin="round"
                class:rotate-180={!open}
        >
        <polyline points="9 18 15 12 9 6" />
        </svg>
    </button>

    <!-- 侧边栏主体 -->
    <aside
            class="fixed top-0 left-0 h-full z-40 bg-base-200 px-4 overflow-auto shadow-lg transform transition-transform duration-300"
            class:translate-x-0={open}
            class:-translate-x-full={!open}
            aria-hidden={!open}
            style="width: {drawerWidthRem}rem;"
    >
        <Interface bind:jointInfos bind:selectedUpAxis />
    </aside>
</div>

<style>
    :global(.translate-x-0) {
        transform: translateX(0);
    }
    :global(.-translate-x-full) {
        transform: translateX(-100%);
    }
</style>
