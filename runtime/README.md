# Runtime State

Project-owned process IDs, sockets, generated runtimes, and temporary state
belong here. The directory may be cleared only while all project processes are
stopped.

`./optflow build-lio` creates the ignored `runtime/lio/` tree. It contains the
pinned no-root ROS 2 Humble arm64 runtime, minimal PCL, ROS conversion packages,
and Hesai FAST-LIO2 binary. Rebuild it with the command after clearing it; never
copy a ROS install from another workspace.
