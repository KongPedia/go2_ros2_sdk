// pybind_module_v2.cpp -- Python bindings for V2 processing (P3 SIMD optimisation).
// Exposes process_u8_to_xyzi_f32_v2 with the same Python API as V1
// so benchmarks can call both without code changes.

#include "lidar_accelerator/processing_v2.hpp"
#include "lidar_accelerator/wasm_decode_v2.hpp"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <pybind11/gil.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(lidar_accelerator_v2, m)
{
  m.doc() = "P3 SIMD-optimised LiDAR processing (two-phase SoA + NEON auto-vectorisation)";

  m.def(
    "decode_and_process",
    [](py::bytes compressed,
      float res,
      py::sequence origin,
      float intense_limiter,
      bool deduplicate,
      int downsample_step,
      int max_points) -> py::array_t<float>
    {
      std::string c = compressed;
      if (origin.size() != 3) {
        throw std::runtime_error("origin must have length 3");
      }
      float origin_f[3] = {
        origin[0].cast<float>(),
        origin[1].cast<float>(),
        origin[2].cast<float>(),
      };
      std::size_t out_points = 0;
      std::vector<float> out;
      {
        py::gil_scoped_release release;
        out = lidar_accelerator::decode_and_process_v2(
          reinterpret_cast<const uint8_t *>(c.data()),
          static_cast<std::size_t>(c.size()),
          res,
          origin_f,
          intense_limiter,
          deduplicate,
          downsample_step,
          max_points,
          &out_points);
      }
      std::vector<py::ssize_t> shape = {
        static_cast<py::ssize_t>(out_points),
        static_cast<py::ssize_t>(4),
      };
      py::array_t<float> out_arr(shape);
      if (out_points > 0) {
        std::memcpy(out_arr.mutable_data(), out.data(), out.size() * sizeof(float));
      }
      return out_arr;
    },
    py::arg("compressed"),
    py::arg("res"),
    py::arg("origin"),
    py::arg("intense_limiter"),
    py::arg("deduplicate") = true,
    py::arg("downsample_step") = 1,
    py::arg("max_points") = 0,
    "WASM decode + V2 SIMD processing (drop-in replacement for lidar_accelerator.decode_and_process).");

  m.def(
    "process_u8_to_xyzi_f32_v2",
    [](py::object positions,
      py::object uvs,
      float res,
      py::sequence origin,
      float intense_limiter,
      bool deduplicate,
      int downsample_step,
      int max_points) -> py::array_t<float>
    {
      py::array pos_arr = py::array::ensure(positions);
      py::array uv_arr = py::array::ensure(uvs);
      if (!pos_arr || !uv_arr) {
        throw std::runtime_error("positions/uvs must be array-like");
      }

      if (pos_arr.ndim() != 1) {pos_arr = pos_arr.attr("reshape")(-1);}
      if (uv_arr.ndim() != 1) {uv_arr = uv_arr.attr("reshape")(-1);}

      if (pos_arr.dtype().kind() != 'u' || pos_arr.itemsize() != 1) {
        pos_arr = pos_arr.attr("astype")(py::dtype("uint8"));
      }
      if (uv_arr.dtype().kind() != 'u' || uv_arr.itemsize() != 1) {
        uv_arr = uv_arr.attr("astype")(py::dtype("uint8"));
      }

      auto pos_u8 = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>(pos_arr);
      auto uv_u8 = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>(uv_arr);

      if (origin.size() != 3) {
        throw std::runtime_error("origin must have length 3");
      }
      float origin_f[3] = {
        origin[0].cast<float>(),
        origin[1].cast<float>(),
        origin[2].cast<float>(),
      };

      std::size_t out_points = 0;
      std::vector<float> out;
      {
        py::gil_scoped_release release;
        out = lidar_accelerator::process_u8_to_xyzi_f32_v2(
          pos_u8.data(),
          static_cast<size_t>(pos_u8.size()),
          uv_u8.data(),
          static_cast<size_t>(uv_u8.size()),
          res,
          origin_f,
          intense_limiter,
          deduplicate,
          downsample_step,
          max_points,
          &out_points);
      }

      std::vector<py::ssize_t> shape = {
        static_cast<py::ssize_t>(out_points),
        static_cast<py::ssize_t>(4),
      };
      py::array_t<float> out_arr(shape);
      if (out_points > 0) {
        std::memcpy(out_arr.mutable_data(), out.data(), out.size() * sizeof(float));
      }
      return out_arr;
    },
    py::arg("positions"),
    py::arg("uvs"),
    py::arg("res"),
    py::arg("origin"),
    py::arg("intense_limiter"),
    py::arg("deduplicate") = true,
    py::arg("downsample_step") = 1,
    py::arg("max_points") = 0,
    "V2 SIMD-optimised: two-phase SoA conversion (P3). Same semantics as V1.");
}
