# Linux-only Revo2 Examples

This directory contains Linux-specific examples that require libraries not available on macOS/Windows.

## EtherCAT Example

```bash
# Build
make

# Run (requires root or ethercat permissions)
make run revo2_ethercat
```

### Prerequisites

- libethercat.so installed
- EtherCAT master configured
- Appropriate permissions (see `scripts/EtherCAT_permissions.sh`)
