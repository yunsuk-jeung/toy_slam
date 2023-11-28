
#define VKL_DELETE_COPY_CONSTRUCTORS(TypeName)                                           \
  TypeName(const TypeName&)       = delete;                                              \
  void operator=(const TypeName&) = delete

#define VKL_DELETE_MOVE_CONSTRUCTORS(TypeName)                                           \
  TypeName(TypeName&&)       = delete;                                                   \
  void operator=(TypeName&&) = delete