#pragma once

#include <memory>

//These macros were inspired mainly on Maplab's macros
//https://github.com/ethz-asl/maplab

#define USING_SMART_PTR(TypeName)                                                        \
  using Ptr   = std::shared_ptr<TypeName>;                                               \
  using CPtr  = std::shared_ptr<const TypeName>;                                         \
  using Uni   = std::unique_ptr<TypeName>;                                               \
  using Cuni  = std::unique_ptr<const TypeName>;                                         \
  using Weak  = std::weak_ptr<TypeName>;                                                 \
  using CWeak = std::weak_ptr<const TypeName>

#define DELETE_COPY_CONSTRUCTORS(TypeName)                                               \
  TypeName(const TypeName&)       = delete;                                              \
  void operator=(const TypeName&) = delete

#define DELETE_MOVE_CONSTRUCTORS(TypeName)                                               \
  TypeName(TypeName&&)       = delete;                                                   \
  void operator=(TypeName&&) = delete
  