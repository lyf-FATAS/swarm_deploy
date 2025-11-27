#pragma once
#include <vector>
#include <cstddef>

class ObservationQueue
{
public:
    ObservationQueue(std::size_t history_len, std::size_t obs_dim);

    bool push(const std::vector<float>& obs);       // 推入一帧观测（长度必须==obs_dim）
    void reset();                                   // 清空队列（全部置零）
    bool ready() const;                             // 是否已积满 history_len 帧
    std::size_t size() const;                       // 当前已积累的帧数（<= history_len）

    // 将历史展平成一维（从“旧到新”的时间顺序），out.size() 将被设置为 history_len*obs_dim
    void getFlattened(std::vector<float>& out) const;

    // 访问参数
    std::size_t historyLen() const;
    std::size_t obsDim() const;

private:
    std::size_t H_;
    std::size_t D_;

    // 环形缓冲：以行主序存储，逻辑上为 H_ x D_，实际是一维数组
    std::vector<float> buf_;
    std::size_t head_;     // 指向“下一帧写入位置”的行号（0..H_-1）
    std::size_t filled_;   // 已写入的帧数（最多到 H_）
};
