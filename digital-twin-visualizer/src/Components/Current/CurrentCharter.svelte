<script lang="ts">
    import { onMount, onDestroy } from 'svelte';
    import {Chart, registerables} from 'chart.js';

    Chart.register(...registerables);

    export let armName = "机械臂 A";
    export let maxPoints = 50; // 横轴显示的采样点数量

    let canvas: HTMLCanvasElement;
    let chart: Chart;

    // 6个轴的颜色定义
    const colors = [
        '#ff6384', '#36a2eb', '#cc65fe', '#ffce56', '#4bc0c0', '#f67019'
    ];

    onMount(() => {
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: Array(maxPoints).fill(''), // 初始空标签
                datasets: colors.map((color, i) => ({
                    label: `轴 ${i + 1}`,
                    data: Array(maxPoints).fill(0), // 初始全为 0
                    borderColor: color,
                    borderWidth: 2,
                    pointRadius: 0, // 不显示点，只显示线，极大提高性能
                    fill: false,
                    tension: 0.2    // 略微平滑
                }))
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false, // 全局禁用动画
                scales: {
                    y: {
                        type: 'linear',
                        min: 0,
                        max: 1,
                        ticks: {stepSize: 0.2},
                        beginAtZero: true,
                        title: { display: true, text: '电流 (A)' }
                    },
                    x: {
                        display: false // 实时曲线通常隐藏 X 轴标签
                    }
                },
                plugins: {
                    legend: { position: 'top', labels: { boxWidth: 10, fontSize: 10 } }
                }
            }
        });
    });

    // 暴露给父组件调用的更新方法
    export function updateData(currentValues: number[]) {
        if (!chart) return;

        chart.data.datasets.forEach((dataset, index) => {
            const val = currentValues[index] || 0;
            dataset.data.push(val); // 推入新数据
            dataset.data.shift();   // 移除最旧的数据
        });

        // 性能核心：使用 'none' 模式跳过复杂的布局计算和动画
        chart.update('none');
    }

    onDestroy(() => {
        if (chart) chart.destroy();
    });
</script>

<div class="h-full w-full p-2 bg-white rounded shadow-sm flex flex-col">
    <h3 class="text-xs font-bold mb-1 text-gray-500 uppercase flex-none">{armName} 电流监控</h3>
    <div class="relative flex-1 min-h-0 w-full">
        <canvas bind:this={canvas}></canvas>
    </div>
</div>
