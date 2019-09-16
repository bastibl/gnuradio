# GNU Radio Yolo

This is an experimental branch towards a more modern runtime environment for GNU Radio.

## Problem

- **complex runtime environment** that is hard to maintain and extend
- **lack of a scheduler** (GNU Radio just starts one thread per block and leaves actual scheduling to the operating system)

The latter has inherent limitations:

- We cannot control the order in which blocks are scheduled and, therefore, cannot exploit cache coherency.
- We cannot control when and for how long threads are scheduled. A thread might, therefore, be interrupted while holding locks on data, which can lead to suboptimal process sequences.
- We cannot assign multiple blocks to one thread and, therefore, have to use synchronization primitives (like mutexes and semaphores) for all shared data structures.

## Vision

- modular scheduler
- custom buffers to support heterogeneous DSP
- distributed DSP

## Road Map

- **Benchmarking**: We need to tool to evaluate the performance of different approaches. This was already started:

  - Bastian Bloessl, Marcus Müller and Matthias Hollick, “Benchmarking and Profiling the GNU Radio Scheduler,” Proceedings of 9th GNU Radio Conference (GRCon 2019), Huntsville, AL, Sep 2019. [[BibTeX](https://www.bastibl.net/bib/bloessl2019benchmarking/bloessl2019benchmarking.bib), [PDF and Details…](https://www.bastibl.net/bib/bloessl2019benchmarking/)]

There is also a [repository](https://github.com/bastibl/gr-sched) with tools for performance measurements and a  work-in-progress [series of blog posts](https://www.bastibl.net/gnuradio-performance-1/).
What we still lack are more flowgraphs and configurations that are tested on more architectures.

- **Refactoring**: The code base grew organically, is hard to maintain, and hard to extend. We already refactored quite a lot of stuff in this branch. See highlights.
- Use **message passing** for thread synchronization. This simplifies code structure further and makes the following steps possible.
- **Decouple blocks from threads**
- Add **scheduler interface** to allow plugging in different schedulers.
- Work on **advanced scheduling algorithms** Your science goes here.
- **Custom buffers**: native support for FPGA, GPU, and DMA buffers
- **Distributed DSP**: make GNU Radio an SDR systems framework

## Highlights

- The message passing API and implementation was refactored to use standard types instead of PMTs. Now, only the actual message is a PMT. This lead to considerable performance improvements. See the paper cited above for the experiment setup.

![Message Passing Performance](/msg_performance.png?raw=true)

- GNU Radio had multiple message passing interfaces. We removed redundant `gr::message` types and ported the blocks that used them to PMTs.

- Removed some `xxx_detail` and `xxx_impl` that were (1) not consistently used in the code base and (2) unnecessary indirections.

- Clean up OOT inheritance. See for example `message_accepter` in the original code base, to see how bad things can be.

- Get rid of `shared_ptr` magic (used in the hier block).

- Refactor `block` naming mess. (Each block had a name, an alias, a symbolic_id, and identifier, a unique id, a symbol_name, ...)

- Start introducing an ownership model to avoid excessive (ab)use of `shared_ptr`, which introduces overhead (due to atomic operations in the constructor and destructors).

- Rename `hier_block2` to `hier_block`.

- Removed tons of cruft.
