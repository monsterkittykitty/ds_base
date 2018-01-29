//
// Created by zac on 1/25/18.
//

#ifndef DS_BASE_DS_GLOBAL_H
#define DS_BASE_DS_GLOBAL_H

namespace ds_base
{
#define DS_DISABLE_COPY(Class)                                                                                         \
  Class(const Class&) = delete;                                                                                        \
  Class& operator=(const Class&) = delete;

#define DS_DECLARE_PRIVATE(Class)                                                                                      \
  inline Class##Private* d_func()                                                                                      \
  {                                                                                                                    \
    return reinterpret_cast<Class##Private*>(d_ptr_.get());                                                            \
  }                                                                                                                    \
  inline const Class##Private* d_func() const                                                                          \
  {                                                                                                                    \
    return reinterpret_cast<const Class##Private*>(d_ptr_.get());                                                      \
  }                                                                                                                    \
  friend class Class##Private;

#define DS_DECLARE_PRIVATE_D(Dptr, Class)                                                                              \
  inline Class##Private* d_func()                                                                                      \
  {                                                                                                                    \
    return reinterpret_cast<Class##Private*>(Dptr);                                                                    \
  }                                                                                                                    \
  inline const Class##Private* d_func() const                                                                          \
  {                                                                                                                    \
    return reinterpret_cast<const Class##Private*>(Dptr);                                                              \
  }                                                                                                                    \
  friend class Class##Private;

#define DS_DECLARE_PUBLIC(Class)                                                                                       \
  inline Class* q_func()                                                                                               \
  {                                                                                                                    \
    return static_cast<Class*>(q_ptr_);                                                                                \
  }                                                                                                                    \
  inline const Class* q_func() const                                                                                   \
  {                                                                                                                    \
    return static_cast<const Class*>(q_ptr_);                                                                          \
  }                                                                                                                    \
  friend class Class;

#define DS_D(Class) Class##Private* const d = d_func()
#define DS_Q(Class) Class* const q = q_func()
}

#endif  // DS_BASE_DS_GLOBAL_H
