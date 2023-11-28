#pragma once

#include <memory>

//These macros were inspired mainly on Maplab's macros
//https://github.com/ethz-asl/maplab

#define TOY_SMART_PTR(TypeName)                                                          \
  using Ptr   = std::shared_ptr<TypeName>;                                               \
  using CPtr  = std::shared_ptr<const TypeName>;                                         \
  using UPtr  = std::unique_ptr<TypeName>;                                               \
  using UCPtr = std::unique_ptr<const TypeName>;                                         \
  using WPtr  = std::weak_ptr<TypeName>;                                                 \
  using WCPtr = std::weak_ptr<const TypeName>

#define TOY_DELETE_COPY_CONSTRUCTORS(TypeName)                                           \
  TypeName(const TypeName&)       = delete;                                              \
  void operator=(const TypeName&) = delete

#define TOY_DELETE_MOVE_CONSTRUCTORS(TypeName)                                           \
  TypeName(TypeName&&)       = delete;                                                   \
  void operator=(TypeName&&) = delete