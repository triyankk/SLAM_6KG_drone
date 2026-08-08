# Third-Party Source

Estimator and sensor-driver revisions, patches, and provenance selected for
this project are pinned under this directory. Generated source checkouts live
under ignored `runtime/` and must not be imported from sibling workspace
folders.

Each dependency must record its upstream URL, revision, license, local patches,
and supported Jetson/ROS version before it enters the flight runtime.

`patches/fast_lio_hesai-user-runtime.patch` removes the unused `pcl_ros` and
component dependencies from the pinned Hesai ROS 2 branch, adds its actual
`tf2_ros` dependency, and links the PCL filters component used by the source.
The bootstrap script applies it to ignored runtime source and records every
upstream revision in `runtime/lio/runtime-manifest.json`.
