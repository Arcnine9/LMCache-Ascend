#include "mem_alloc.h"
#include "managed_mem.h"
#include <acl/acl.h>
#include <atomic>
#include <cstdlib> // for std::getenv
#include <cstring> // for strerror
#include <errno.h>
#include <iostream>
#include <numaif.h>
#include <string>
#include <sys/mman.h>

uintptr_t alloc_pinned_ptr(std::size_t size, unsigned int flags) {
  void *ptr = nullptr;
  // no flags
  aclError err = aclrtMallocHost(&ptr, size);
  if (err != ACL_SUCCESS) {
    throw std::runtime_error("aclrtMallocHost failed: " + std::to_string(err));
  }

  const char *socVersion = aclrtGetSocName();

  // nullptr means that the chip version failed to be obtained. We cannot be
  // sure about the version of the device. Unless we are sure that we deal with
  // a 310 device, we try to register.
  if (socVersion == nullptr ||
      std::string(socVersion).find("310") == std::string::npos) {
    // not 310p
    // Host-register is a hard dependency of the zero-copy transfer path,
    // but it is unsupported on some drivers (e.g. 24.1.rc3 returns
    // 65534 from halHostRegister and 507899 from aclrtHostRegister).
    // Auto-detect once: if the first registration fails, fall back to
    // unregistered pinned memory; the connector must then use HBM
    // staging transfers instead of zero-copy kernels.
    static std::atomic<bool> register_supported{true};
    if (register_supported.load()) {
      auto devPtr = register_ptr(ptr, size);
      if (devPtr == nullptr) {
        register_supported.store(false);
        std::cerr << "LMCache-Ascend: host-register unsupported by driver, "
                     "falling back to unregistered pinned memory "
                     "(HBM staging transfer path required)."
                  << std::endl;
      }
    }
  }

  return reinterpret_cast<uintptr_t>(ptr);
}

void free_pinned_ptr(uintptr_t ptr) {
  unregister_ptr(reinterpret_cast<void *>(ptr));
  aclError err = aclrtFreeHost(reinterpret_cast<void *>(ptr));
  if (err != ACL_SUCCESS) {
    throw std::runtime_error("aclrtFreeHost failed: " + std::to_string(err));
  }
}

/*
 * This function is potentially slow for the mbind
 */
uintptr_t alloc_pinned_numa_ptr(std::size_t size, int node) {
  void *ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (ptr == MAP_FAILED) {
    throw std::runtime_error(std::string("mmap failed: ") + strerror(errno));
  }

  // Maximum of 64 numa nodes
  unsigned long mask = 1UL << node;
  long maxnode = 8 * sizeof(mask);
  int err = mbind(ptr, size, MPOL_BIND, &mask, maxnode,
                  MPOL_MF_MOVE | MPOL_MF_STRICT);
  if (err != 0) {
    munmap(ptr, size);
    throw std::runtime_error(std::string("mbind failed: ") + strerror(errno));
  }

  memset(ptr, 0, size);

  // as before we need to actually save the dev ptr for later reuse,
  // because acl APIs do not allow retrieving register dev ptr.
  // Same auto-detect fallback as alloc_pinned_ptr: tolerate drivers
  // without host-register support (see alloc_pinned_ptr).
  static std::atomic<bool> numa_register_supported{true};
  if (numa_register_supported.load()) {
    auto devPtr = register_ptr(ptr, size);
    if (devPtr == nullptr) {
      numa_register_supported.store(false);
      std::cerr << "LMCache-Ascend: host-register unsupported by driver, "
                   "falling back to unregistered NUMA host memory "
                   "(HBM staging transfer path required)."
                << std::endl;
    }
  }

  return reinterpret_cast<uintptr_t>(ptr);
}

void free_pinned_numa_ptr(uintptr_t p, std::size_t size) {
  void *ptr = reinterpret_cast<void *>(p);

  auto unRegErr = unregister_ptr(ptr);
  auto unMapErr = munmap(ptr, size);
  if (unRegErr) {
    throw std::runtime_error("unregister_ptr failed: " +
                             std::to_string(unRegErr));
  }
  if (unMapErr) {
    throw std::runtime_error("munmap failed: " + std::to_string(unMapErr));
  }
}
