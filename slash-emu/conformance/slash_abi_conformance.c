/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * SLASH filesystem ABI conformance suite -- the single source of truth.
 *
 * This is THE black-box conformance test for the new SLASH filesystem ABI
 * (abi_rebuild_architecture.md "Testing and Code quality").  It is deliberately
 * the ONE exception to the project's GTest rule: it uses the kernel's kselftest
 * harness so that the SAME, UNCHANGED suite can later run against the in-kernel
 * SLASH filesystem (architecture step 6), not only the FUSE emulation daemon.
 *
 * To make that possible it is a PURE black-box test over the mount path:
 *
 *   - It uses ONLY the shared UAPI (slash/uapi/slash_abi.h) and raw syscalls
 *     (open/read/pread/pwrite/ioctl/unlink/mmap/readdir/lseek) against files
 *     under a parameterised mount root.
 *   - It includes NO daemon-internal headers, NO libfuse, and assumes NOTHING
 *     about the backend being FUSE.  Pointing it at a kernel mount is just a
 *     matter of changing the mount-root env var and bringing up that backend.
 *
 * Parameterisation (backend-neutral):
 *   - SLASH_CONFORMANCE_MOUNT   mount root (default /run/slash_emu).  The device
 *                               under test is DISCOVERED by readdir of this root
 *                               (the single "DDDD:BB:DD" directory), so the suite
 *                               never hard-codes a BDF.
 *   - SLASH_CONFORMANCE_STUB_MODEL  path to a model executable to pack into the
 *                               CI "VBIN" for the reconfiguration round-trip.  If
 *                               unset, the reconfiguration tests SKIP (a kernel
 *                               backend has no VBIN-stub concept; only the FUSE
 *                               daemon does).
 *
 * The CTest runner (run_conformance.sh) brings up the backend (the FUSE daemon
 * via emud-scratch.sh), points SLASH_CONFORMANCE_MOUNT at the live mount, runs
 * this binary, and tears the backend down.  See conformance/README.md for how to
 * retarget a different backend (e.g. the future kernel module).
 */

#define _GNU_SOURCE

#include "kselftest_harness.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <slash/uapi/slash_abi.h>

/* ───────────────────────────── parameterisation ───────────────────────────── */

static const char *mount_root(void)
{
	const char *m = getenv("SLASH_CONFORMANCE_MOUNT");

	return (m != NULL && m[0] != '\0') ? m : "/run/slash_emu";
}

static const char *stub_model_path(void)
{
	const char *p = getenv("SLASH_CONFORMANCE_STUB_MODEL");

	return (p != NULL && p[0] != '\0') ? p : NULL;
}

/*
 * Discover the device directory by readdir of the mount root: pick the single
 * entry that is NOT the global "hotplug" file (and not . / ..).  This keeps the
 * suite backend-neutral -- it never hard-codes a BDF; it learns the configured
 * one from the mount.  Returns 0 on success (fills bdf_out, size at least
 * SLASH_PCI_BDF_LEN), -errno otherwise.
 */
static int discover_bdf(char *bdf_out, size_t cap)
{
	DIR *d = opendir(mount_root());
	struct dirent *e;
	int found = 0;

	if (d == NULL)
		return -errno;

	while ((e = readdir(d)) != NULL) {
		if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0)
			continue;
		if (strcmp(e->d_name, "hotplug") == 0)
			continue;
		/* First non-hotplug entry is the device directory. */
		if (!found) {
			snprintf(bdf_out, cap, "%s", e->d_name);
			found = 1;
		}
	}
	closedir(d);
	return found ? 0 : -ENOENT;
}

/* Build "<mount>/<bdf>/<suffix>" into buf. */
static void dev_path(char *buf, size_t cap, const char *bdf, const char *suffix)
{
	if (suffix != NULL && suffix[0] != '\0')
		snprintf(buf, cap, "%s/%s/%s", mount_root(), bdf, suffix);
	else
		snprintf(buf, cap, "%s/%s", mount_root(), bdf);
}

/* ───────────────────────────── small helpers ──────────────────────────────── */

static void fill_pattern(uint8_t *buf, size_t len, uint8_t seed)
{
	for (size_t i = 0; i < len; i++)
		buf[i] = (uint8_t) ((i + seed) & 0xff);
}

/* Open the qdma/ dir, QPAIR_ADD (MM, H2C|C2H), open the qpair file.  Returns the
 * qpair fd (>=0) or -1; *qid_out is set on success. */
static int open_qpair(const char *bdf, uint32_t *qid_out)
{
	char path[4096];
	struct slash_abi_qdma_qpair_add req;
	int dfd, rc, qfd;

	dev_path(path, sizeof(path), bdf, "qdma");
	dfd = open(path, O_RDONLY | O_DIRECTORY);
	if (dfd < 0)
		return -1;

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);
	req.mode = 0;       /* MM */
	req.dir_mask = 0x3; /* H2C | C2H */
	rc = ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req);
	close(dfd);
	if (rc != 0)
		return -1;

	if (qid_out != NULL)
		*qid_out = req.qid;
	dev_path(path, sizeof(path), bdf, "qdma");
	snprintf(path + strlen(path), sizeof(path) - strlen(path), "/qpair%u",
		 req.qid);
	qfd = open(path, O_RDWR);
	return qfd;
}

/* Issue a hotplug device-request ioctl (REMOVE/SBR/HOTPLUG).  Returns 0 or
 * -errno. */
static int hotplug_dev_ioctl(unsigned long cmd, const char *bdf_func)
{
	char path[4096];
	struct slash_abi_hotplug_device_request req;
	int fd, rc, saved;

	snprintf(path, sizeof(path), "%s/hotplug", mount_root());
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);
	snprintf(req.bdf, sizeof(req.bdf), "%s", bdf_func);
	rc = ioctl(fd, cmd, &req);
	saved = errno;
	close(fd);
	return rc == 0 ? 0 : -saved;
}

/* RESCAN takes no struct. */
static int hotplug_rescan(void)
{
	char path[4096];
	int fd, rc, saved;

	snprintf(path, sizeof(path), "%s/hotplug", mount_root());
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;
	rc = ioctl(fd, SLASH_ABI_HOTPLUG_IOCTL_RESCAN);
	saved = errno;
	close(fd);
	return rc == 0 ? 0 : -saved;
}

/* ───────────────── minimal CI "VBIN" (ustar archive) builder ───────────────── */

/*
 * Build the minimal VBIN the bridge accepts: a POSIX ustar archive carrying the
 * model executable as a member named "vpp_sim" (bridge-protocol.md §3.1).  This
 * matches the tar layout the daemon/integration tests already agree on.  Returns
 * a malloc'd buffer (*len_out set) or NULL.
 */
static void tar_put_member(uint8_t *buf, size_t hoff, const char *name,
			   const uint8_t *content, size_t clen, unsigned mode)
{
	uint8_t *h = buf + hoff;
	unsigned sum = 0;

	memset(h, 0, 512);
	snprintf((char *) h, 100, "%s", name);
	snprintf((char *) (h + 100), 8, "%07o", mode & 07777);
	snprintf((char *) (h + 108), 8, "%07o", 0);
	snprintf((char *) (h + 116), 8, "%07o", 0);
	snprintf((char *) (h + 124), 12, "%011o", (unsigned) clen);
	snprintf((char *) (h + 136), 12, "%011o", 0);
	h[156] = '0'; /* typeflag: regular file */
	memcpy(h + 257, "ustar", 5);
	h[263] = '0';
	h[264] = '0';
	memset(h + 148, ' ', 8); /* checksum field blanked for sum */
	for (int i = 0; i < 512; i++)
		sum += h[i];
	snprintf((char *) (h + 148), 8, "%06o", sum);
	h[154] = '\0';
	h[155] = ' ';
	memcpy(h + 512, content, clen);
}

static uint8_t *make_ci_vbin(const char *model_path, size_t *len_out)
{
	FILE *f = fopen(model_path, "rb");
	long sz;
	uint8_t *content, *out;
	size_t body, total;

	if (f == NULL)
		return NULL;
	fseek(f, 0, SEEK_END);
	sz = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (sz <= 0) {
		fclose(f);
		return NULL;
	}
	content = malloc((size_t) sz);
	if (content == NULL) {
		fclose(f);
		return NULL;
	}
	if (fread(content, 1, (size_t) sz, f) != (size_t) sz) {
		free(content);
		fclose(f);
		return NULL;
	}
	fclose(f);

	body = ((size_t) sz + 511) / 512 * 512;
	total = 512 + body + 1024; /* header + padded body + 2 zero blocks */
	out = calloc(1, total);
	if (out == NULL) {
		free(content);
		return NULL;
	}
	tar_put_member(out, 0, "vpp_sim", content, (size_t) sz, 0755);
	free(content);
	*len_out = total;
	return out;
}

/*
 * Build a LARGE multi-member VBIN: the real model packed as "vpp_sim" plus a
 * sibling padding member large enough that the whole ustar archive exceeds
 * `min_total` bytes.  This is what forces the T10 chunk-reassembly path: a single
 * pwrite(2) of a payload larger than the FUSE max_write is split by the kernel
 * into several contiguous in-region writes, so the daemon only sees the complete
 * VBIN if it correctly reassembles them.  The padding member is realistic VBIN
 * ballast (a sim VBIN carries sibling .so's / system_map.xml alongside vpp_sim).
 * Returns a malloc'd buffer (*len_out set) or NULL.
 */
static uint8_t *make_large_vbin(const char *model_path, size_t min_total,
				size_t *len_out)
{
	FILE *f = fopen(model_path, "rb");
	long sz;
	uint8_t *content;
	size_t model_body, pad_clen, pad_body, total;
	uint8_t *out;

	if (f == NULL)
		return NULL;
	fseek(f, 0, SEEK_END);
	sz = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (sz <= 0) {
		fclose(f);
		return NULL;
	}
	content = malloc((size_t) sz);
	if (content == NULL) {
		fclose(f);
		return NULL;
	}
	if (fread(content, 1, (size_t) sz, f) != (size_t) sz) {
		free(content);
		fclose(f);
		return NULL;
	}
	fclose(f);

	model_body = ((size_t) sz + 511) / 512 * 512;
	/* Size the padding member so the archive clears min_total comfortably. */
	pad_clen = min_total + (1u << 20);
	pad_body = (pad_clen + 511) / 512 * 512;
	total = 512 + model_body + 512 + pad_body + 1024;
	out = calloc(1, total);
	if (out == NULL) {
		free(content);
		return NULL;
	}

	/* Member 1: the real model (so the reassembled archive is RUNNABLE). */
	tar_put_member(out, 0, "vpp_sim", content, (size_t) sz, 0755);
	free(content);

	/* Member 2: ballast.  A deterministic byte pattern (not all-zero, so it is
	 * never mistaken for the archive terminator mid-body). */
	uint8_t *pad = malloc(pad_clen);
	if (pad == NULL) {
		free(out);
		return NULL;
	}
	fill_pattern(pad, pad_clen, 0xC3);
	tar_put_member(out, 512 + model_body, "ballast.bin", pad, pad_clen, 0644);
	free(pad);

	*len_out = total;
	return out;
}

/* ───────────────────────────────── info ───────────────────────────────────── */
/*
 * Contract (slash_abi.h + architecture): info is read-only binary slash_info.
 * `size` is [out] (a plain read(2) has no [in] channel).  A full read yields
 * acc_type with SYSTEM_EMULATED set (for the emulated backend) and a board-level
 * bdf matching the directory.  A short read is a valid prefix; offset reads work;
 * a read at/after EOF returns 0.
 */

FIXTURE(info)
{
	char bdf[SLASH_PCI_BDF_LEN];
	char path[4096];
	int fd;
};

FIXTURE_SETUP(info)
{
	self->fd = -1;
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
	dev_path(self->path, sizeof(self->path), self->bdf, "info");
	self->fd = open(self->path, O_RDONLY);
	ASSERT_GE(self->fd, 0)
	TH_LOG("open %s: %s", self->path, strerror(errno));
}

FIXTURE_TEARDOWN(info)
{
	if (self->fd >= 0)
		close(self->fd);
}

TEST_F(info, full_read_struct)
{
	struct slash_info in;
	ssize_t n;

	memset(&in, 0, sizeof(in));
	n = read(self->fd, &in, sizeof(in));
	ASSERT_EQ(n, (ssize_t) sizeof(in))
	TH_LOG("read info: %zd (%s)", n, strerror(errno));

	/* size is [out]: the producer populates it with its struct size. */
	EXPECT_EQ(in.size, (uint32_t) sizeof(struct slash_info));
	/* The emulated backend asserts SYSTEM_EMULATED.  (A kernel backend would
	 * clear it; this is the conformance-relevant invariant for the daemon.) */
	EXPECT_NE(in.acc_type & SLASH_ACC_TYPE_SYSTEM_EMULATED, 0u)
	TH_LOG("acc_type=0x%x (expected SYSTEM_EMULATED set)", in.acc_type);
	/* bdf is board-level (no function) and matches the directory name. */
	EXPECT_STREQ(in.bdf, self->bdf);
}

TEST_F(info, file_size_matches_struct)
{
	struct stat st;

	ASSERT_EQ(fstat(self->fd, &st), 0);
	EXPECT_EQ((size_t) st.st_size, sizeof(struct slash_info))
	TH_LOG("info file size %lld != sizeof(slash_info) %zu",
	       (long long) st.st_size, sizeof(struct slash_info));
}

TEST_F(info, short_read_is_valid_prefix)
{
	struct slash_info full;
	uint32_t prefix = 0;
	ssize_t n;

	memset(&full, 0, sizeof(full));
	ASSERT_EQ(read(self->fd, &full, sizeof(full)), (ssize_t) sizeof(full));

	/* A short read of only the leading size field yields the same value. */
	ASSERT_EQ(lseek(self->fd, 0, SEEK_SET), 0);
	n = read(self->fd, &prefix, sizeof(prefix));
	ASSERT_EQ(n, (ssize_t) sizeof(prefix));
	EXPECT_EQ(prefix, full.size);
}

TEST_F(info, offset_read)
{
	struct slash_info full;
	uint32_t acc = 0;
	ssize_t n;

	memset(&full, 0, sizeof(full));
	ASSERT_EQ(read(self->fd, &full, sizeof(full)), (ssize_t) sizeof(full));

	/* pread the acc_type field at its offset. */
	n = pread(self->fd, &acc, sizeof(acc),
		  (off_t) offsetof(struct slash_info, acc_type));
	ASSERT_EQ(n, (ssize_t) sizeof(acc));
	EXPECT_EQ(acc, full.acc_type);
}

TEST_F(info, read_at_eof_returns_zero)
{
	uint8_t b[16];

	EXPECT_EQ(pread(self->fd, b, sizeof(b),
			(off_t) sizeof(struct slash_info)),
		  0);
}

TEST_F(info, read_past_eof_returns_zero)
{
	uint8_t b[16];

	EXPECT_EQ(pread(self->fd, b, sizeof(b),
			(off_t) (sizeof(struct slash_info) + 4096)),
		  0);
}

/* ───────────────────────────────── bars ───────────────────────────────────── */
/*
 * Contract: bar0/bar2/bar4 exist with sizes equal to the file size; bar1/3/5 are
 * absent (ENOENT).  pread/pwrite round-trip; widths outside {1,2,4,8} or
 * misaligned are -EINVAL; out-of-range is rejected (not clamped); an in-range
 * virgin read is 0 (never -EIO).  mmap: MAP_SHARED fails at mmap() with ENODEV;
 * MAP_PRIVATE may map but a deref must fault promptly (SIGBUS) and never hang.
 */

FIXTURE(bars)
{
	char bdf[SLASH_PCI_BDF_LEN];
};

FIXTURE_SETUP(bars)
{
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
}

FIXTURE_TEARDOWN(bars) {}

static void bar_path(char *buf, size_t cap, const char *bdf, unsigned m)
{
	char suffix[32];

	snprintf(suffix, sizeof(suffix), "bars/bar%u", m);
	dev_path(buf, cap, bdf, suffix);
}

/* Format "<board-bdf>.<func>" into buf (used by the revocation/hotplug tests). */
static void bdf_func(char *buf, size_t cap, const char *board, int func)
{
	snprintf(buf, cap, "%s.%d", board, func);
}

TEST_F(bars, present_bars_sizes_match)
{
	const struct {
		unsigned idx;
		uint64_t size;
	} expect[] = {
		{ SLASH_BAR_USER_IDX, SLASH_BAR_USER_SIZE },
		{ SLASH_BAR_SL_IDX, SLASH_BAR_SL_SIZE },
		{ SLASH_BAR_CLK_IDX, SLASH_BAR_CLK_SIZE },
	};

	for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
		char path[4096];
		struct stat st;

		bar_path(path, sizeof(path), self->bdf, expect[i].idx);
		ASSERT_EQ(stat(path, &st), 0)
		TH_LOG("stat %s: %s", path, strerror(errno));
		EXPECT_EQ((uint64_t) st.st_size, expect[i].size)
		TH_LOG("bar%u size %lld != %llu", expect[i].idx,
		       (long long) st.st_size,
		       (unsigned long long) expect[i].size);
	}
}

TEST_F(bars, absent_bars_enoent)
{
	const unsigned absent[] = { 1, 3, 5 };

	for (size_t i = 0; i < sizeof(absent) / sizeof(absent[0]); i++) {
		char path[4096];
		int fd;

		bar_path(path, sizeof(path), self->bdf, absent[i]);
		fd = open(path, O_RDWR);
		EXPECT_EQ(fd, -1)
		TH_LOG("bar%u must be absent", absent[i]);
		EXPECT_EQ(errno, ENOENT);
		if (fd >= 0)
			close(fd);
	}
}

TEST_F(bars, register_round_trip)
{
	char path[4096];
	int fd;
	const uint32_t widths[] = { 1, 2, 4, 8 };

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	for (size_t i = 0; i < sizeof(widths) / sizeof(widths[0]); i++) {
		uint64_t w = 0, r = 0;
		uint32_t width = widths[i];
		off_t off = (off_t) (0x40 * (i + 1)); /* aligned to >= width */

		memcpy(&w, "ABCDEFGH", width);
		ASSERT_EQ(pwrite(fd, &w, width, off), (ssize_t) width)
		TH_LOG("pwrite width=%u: %s", width, strerror(errno));
		ASSERT_EQ(pread(fd, &r, width, off), (ssize_t) width)
		TH_LOG("pread width=%u: %s", width, strerror(errno));
		EXPECT_EQ(memcmp(&w, &r, width), 0)
		TH_LOG("round-trip mismatch width=%u", width);
	}
	close(fd);
}

/*
 * The revised masterplan contract: bar files support plain read()/write() at the
 * IMPLICIT file position, not only pread/pwrite.  llseek to an aligned offset,
 * then issue sequential 4-byte write()s -- each advancing the position by one
 * register -- and read the same registers back with sequential read()s after
 * seeking the position back.  This is the streamed-register pattern VRTD uses.
 */
TEST_F(bars, position_write_read_round_trip)
{
	char path[4096];
	int fd;
	const off_t base = 0x40;
	const uint32_t regs[] = { 0x11111111u, 0x22222222u, 0x33333333u,
				  0x44444444u };
	const size_t nregs = sizeof(regs) / sizeof(regs[0]);

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR | O_SYNC);
	ASSERT_GE(fd, 0);

	/* Stream the registers out with sequential write()s from an aligned base. */
	ASSERT_EQ(lseek(fd, base, SEEK_SET), base)
	TH_LOG("lseek to base: %s", strerror(errno));
	for (size_t i = 0; i < nregs; i++) {
		uint32_t v = regs[i];

		ASSERT_EQ(write(fd, &v, sizeof(v)), (ssize_t) sizeof(v))
		TH_LOG("streamed write reg %zu: %s", i, strerror(errno));
	}

	/* Read them back with sequential read()s from the same aligned base. */
	ASSERT_EQ(lseek(fd, base, SEEK_SET), base);
	for (size_t i = 0; i < nregs; i++) {
		uint32_t v = 0;

		ASSERT_EQ(read(fd, &v, sizeof(v)), (ssize_t) sizeof(v))
		TH_LOG("streamed read reg %zu: %s", i, strerror(errno));
		EXPECT_EQ(v, regs[i])
		TH_LOG("streamed register %zu round-trip mismatch", i);
	}

	close(fd);
}

/*
 * The masterplan's headline example (system_emulation_masterplan.md, BAR usage
 * pattern): llseek to the first parameter register, stream one 4-byte write() per
 * parameter (advancing the position), then issue a SINGLE pwrite() to a control
 * register at a DIFFERENT offset to "start the kernel".  The pwrite must NOT
 * disturb the streamed file position, so a subsequent write() lands on the NEXT
 * parameter register, not back at the control register.
 */
TEST_F(bars, streamed_writes_then_pwrite_control_keeps_position)
{
	char path[4096];
	int fd;
	const off_t ctrl = 0x10000;  /* control register */
	const off_t param = 0x10004; /* first parameter register */
	uint32_t p0 = 0xaaaa0000u, p1 = 0xaaaa0001u, p2 = 0xaaaa0002u;
	uint32_t start = 0x1u;
	uint32_t rb = 0;

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR | O_SYNC);
	ASSERT_GE(fd, 0);

	/* Stream the first two parameters with position-advancing write()s. */
	ASSERT_EQ(lseek(fd, param, SEEK_SET), param);
	ASSERT_EQ(write(fd, &p0, sizeof(p0)), (ssize_t) sizeof(p0));
	ASSERT_EQ(write(fd, &p1, sizeof(p1)), (ssize_t) sizeof(p1));

	/* A pwrite to the control register at a DIFFERENT offset starts the kernel
	 * without disturbing the streamed position. */
	ASSERT_EQ(pwrite(fd, &start, sizeof(start), ctrl), (ssize_t) sizeof(start))
	TH_LOG("control pwrite: %s", strerror(errno));

	/* The next streamed write() must land on the THIRD parameter (param + 8),
	 * proving the pwrite did not move the implicit position. */
	ASSERT_EQ(write(fd, &p2, sizeof(p2)), (ssize_t) sizeof(p2));

	/* Verify: param+0/+4/+8 hold p0/p1/p2 and the control register holds start. */
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), param), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, p0);
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), param + 4), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, p1);
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), param + 8), (ssize_t) sizeof(rb))
	TH_LOG("third streamed write landed at the wrong position");
	EXPECT_EQ(rb, p2);
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), ctrl), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, start);

	close(fd);
}

/*
 * lseek SEEK_SET and SEEK_CUR reposition the implicit file offset correctly: a
 * write() after a seek lands where the seek pointed, and SEEK_CUR is relative to
 * the position the previous I/O advanced to.
 */
TEST_F(bars, lseek_set_and_cur_reposition)
{
	char path[4096];
	int fd;
	uint32_t a = 0xdead0001u, b = 0xdead0002u, rb = 0;

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR | O_SYNC);
	ASSERT_GE(fd, 0);

	/* SEEK_SET to an aligned offset, write -> the byte lands there. */
	ASSERT_EQ(lseek(fd, 0x80, SEEK_SET), 0x80);
	ASSERT_EQ(write(fd, &a, sizeof(a)), (ssize_t) sizeof(a));
	/* The write advanced the position to 0x84; SEEK_CUR by +4 -> 0x88. */
	ASSERT_EQ(lseek(fd, 4, SEEK_CUR), 0x88);
	ASSERT_EQ(write(fd, &b, sizeof(b)), (ssize_t) sizeof(b));

	/* SEEK_CUR can also be used to query the current position (offset 0). */
	ASSERT_EQ(lseek(fd, 0, SEEK_CUR), 0x8c);

	ASSERT_EQ(pread(fd, &rb, sizeof(rb), 0x80), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, a);
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), 0x88), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, b);
	/* The 4-byte gap at 0x84 was skipped by the seek, never written -> zero. */
	ASSERT_EQ(pread(fd, &rb, sizeof(rb), 0x84), (ssize_t) sizeof(rb));
	EXPECT_EQ(rb, 0u);

	close(fd);
}

/*
 * The width {1,2,4,8} + alignment gate that guards pread/pwrite applies EQUALLY
 * to position-based read()/write(): a 4096-byte read() is rejected (-EINVAL, not
 * a clamped short read), a 3-byte read() is rejected, and a width-4 read() at a
 * 2-byte-misaligned position is rejected.  This pins that switching from the
 * explicit-offset to the implicit-position path does not bypass the gate.
 */
TEST_F(bars, position_bad_width_and_alignment_einval)
{
	char path[4096];
	int fd;
	uint8_t buf[4096] = { 0 };

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR | O_SYNC);
	ASSERT_GE(fd, 0);

	/* A 4096-byte read at an aligned position: width not in {1,2,4,8}. */
	ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);
	errno = 0;
	EXPECT_EQ(read(fd, buf, 4096), -1)
	TH_LOG("4096-byte position read should be -EINVAL");
	EXPECT_EQ(errno, EINVAL);

	/* A 3-byte read at an aligned position: width not in {1,2,4,8}. */
	ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);
	errno = 0;
	EXPECT_EQ(read(fd, buf, 3), -1)
	TH_LOG("3-byte position read should be -EINVAL");
	EXPECT_EQ(errno, EINVAL);

	/* A width-4 read at a 2-byte-misaligned position. */
	ASSERT_EQ(lseek(fd, 2, SEEK_SET), 2);
	errno = 0;
	EXPECT_EQ(read(fd, buf, 4), -1)
	TH_LOG("width-4 read at misaligned position should be -EINVAL");
	EXPECT_EQ(errno, EINVAL);

	close(fd);
}

/*
 * Non-{1,2,4,8} transfer widths are rejected with -EINVAL.  Note width 0 is NOT
 * tested here: a 0-length pread/pwrite is a POSIX no-op that returns 0 at the
 * syscall/VFS layer before any read/write reaches the backend, so it is not an
 * observable ABI rejection over the mount path (the daemon's internal width
 * check still rejects 0, but that path is unreachable by a real 0-byte syscall).
 * The tested widths all carry a non-zero, non-power-of-two-≤8 length that DOES
 * reach the backend's width gate.
 */
TEST_F(bars, bad_width_einval)
{
	char path[4096];
	int fd;
	const uint32_t bad[] = { 3, 5, 6, 7, 16 };
	uint8_t buf[32] = { 0 };

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		EXPECT_EQ(pread(fd, buf, bad[i], 0), -1)
		TH_LOG("pread width=%u should be -EINVAL", bad[i]);
		EXPECT_EQ(errno, EINVAL);
		EXPECT_EQ(pwrite(fd, buf, bad[i], 0), -1)
		TH_LOG("pwrite width=%u should be -EINVAL", bad[i]);
		EXPECT_EQ(errno, EINVAL);
	}
	close(fd);
}

TEST_F(bars, misaligned_einval)
{
	char path[4096];
	int fd;
	uint8_t buf[8] = { 0 };

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	/* width 4 at offset 2 is misaligned. */
	EXPECT_EQ(pread(fd, buf, 4, 2), -1);
	EXPECT_EQ(errno, EINVAL);
	EXPECT_EQ(pwrite(fd, buf, 4, 2), -1);
	EXPECT_EQ(errno, EINVAL);
	/* width 8 at offset 4 is misaligned. */
	EXPECT_EQ(pread(fd, buf, 8, 4), -1);
	EXPECT_EQ(errno, EINVAL);
	close(fd);
}

TEST_F(bars, out_of_range_rejected_not_clamped)
{
	char path[4096];
	int fd;
	uint8_t buf[8] = { 0 };

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_CLK_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	/* An 8-byte read straddling the end of the 512 KiB BAR is rejected, not
	 * clamped to a short read. */
	EXPECT_EQ(pread(fd, buf, 8, (off_t) (SLASH_BAR_CLK_SIZE - 4)), -1);
	EXPECT_EQ(errno, EINVAL);
	/* A 4-byte read wholly past the end is rejected. */
	EXPECT_EQ(pread(fd, buf, 4, (off_t) SLASH_BAR_CLK_SIZE), -1);
	EXPECT_EQ(errno, EINVAL);
	/* The last valid aligned 8-byte read succeeds. */
	EXPECT_EQ(pread(fd, buf, 8, (off_t) (SLASH_BAR_CLK_SIZE - 8)), 8);
	close(fd);
}

TEST_F(bars, virgin_read_is_zero_never_eio)
{
	char path[4096];
	int fd;
	uint64_t v = 0xdeadbeefcafef00dULL;

	/* BAR2 (service layer) is served from the shadow; a never-written, in-range
	 * register reads as zero -- never -EIO. */
	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_SL_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);
	ASSERT_EQ(pread(fd, &v, 8, (off_t) 0x1000), 8)
	TH_LOG("virgin read failed: %s", strerror(errno));
	EXPECT_EQ(v, 0ULL);
	close(fd);
}

TEST_F(bars, mmap_shared_rejected_enodev)
{
	char path[4096];
	int fd;
	void *p;

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	errno = 0;
	p = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	EXPECT_EQ(p, MAP_FAILED);
	EXPECT_EQ(errno, ENODEV);
	if (p != MAP_FAILED)
		munmap(p, 4096);

	errno = 0;
	p = mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, 0);
	EXPECT_EQ(p, MAP_FAILED);
	EXPECT_EQ(errno, ENODEV);
	if (p != MAP_FAILED)
		munmap(p, 4096);

	close(fd);
}

/*
 * MAP_PRIVATE may succeed at mmap() (the kernel page-cache COW path cannot be
 * vetoed by a userspace FUSE op), but a deref MUST reach a terminal state
 * promptly -- it must NOT hang and must NOT yield device data.  We deref in a
 * CHILD under a watchdog so a hang FAILS (the watchdog SIGKILLs and we report
 * failure) rather than wedging the suite.  The harness also bounds each test
 * with its own alarm() as an outer backstop.
 */
TEST_F(bars, mmap_private_deref_faults_promptly_never_hangs)
{
	char path[4096];
	pid_t child;
	int status = 0;
	int waited = 0;
	const int timeout_ms = 4000;

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);

	child = fork();
	ASSERT_NE(child, -1);
	if (child == 0) {
		/* Restore default SIGBUS so the fault terminates us by signal. */
		signal(SIGBUS, SIG_DFL);
		int fd = open(path, O_RDONLY);
		if (fd < 0)
			_exit(10);
		void *p = mmap(NULL, 4096, PROT_READ, MAP_PRIVATE, fd, 0);
		if (p == MAP_FAILED)
			_exit(0); /* refused outright -- ideal. */
		volatile char c = *(volatile char *) p;
		(void) c;
		_exit(10); /* deref returned (served something) without faulting. */
	}

	while (waited < timeout_ms) {
		if (waitpid(child, &status, WNOHANG) == child)
			break;
		usleep(50 * 1000);
		waited += 50;
	}
	if (waited >= timeout_ms) {
		kill(child, SIGKILL);
		waitpid(child, &status, 0);
		ASSERT_TRUE(0)
		TH_LOG("MAP_PRIVATE deref HUNG (watchdog had to SIGKILL)");
	}
	/* Any prompt terminal state passes; if it died by signal it must be
	 * SIGBUS (a prompt fault), not something else. */
	if (WIFSIGNALED(status)) {
		EXPECT_EQ(WTERMSIG(status), SIGBUS)
		TH_LOG("expected prompt SIGBUS, got signal %d",
		       WTERMSIG(status));
	} else {
		EXPECT_EQ(status, 0)
		TH_LOG("expected a zero return value, got %d", status);
	}
}

/* ───────────────────────────────── qdma ───────────────────────────────────── */
/*
 * Contract: QPAIR_ADD (MM, dir_mask) yields a QID and the qpair<Q> file appears.
 * pread/pwrite MM round-trip into HBM and DDR.  Out-of-range -> -ERANGE.  ST mode
 * and CMPT -> -EOPNOTSUPP.  MM only (streaming deferred -- not tested).
 */

FIXTURE(qdma)
{
	char bdf[SLASH_PCI_BDF_LEN];
	int qfd;
	uint32_t qid;
};

FIXTURE_SETUP(qdma)
{
	self->qfd = -1;
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
}

FIXTURE_TEARDOWN(qdma)
{
	if (self->qfd >= 0)
		close(self->qfd);
}

TEST_F(qdma, qpair_add_creates_file)
{
	char path[4096];
	struct stat st;

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0)
	TH_LOG("open_qpair: %s", strerror(errno));

	dev_path(path, sizeof(path), self->bdf, "qdma");
	snprintf(path + strlen(path), sizeof(path) - strlen(path), "/qpair%u",
		 self->qid);
	EXPECT_EQ(stat(path, &st), 0)
	TH_LOG("qpair%u file should exist", self->qid);
}

TEST_F(qdma, mm_round_trip_hbm)
{
	uint8_t out[512], in[512];

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0);

	fill_pattern(out, sizeof(out), 0x11);
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pwrite(self->qfd, out, sizeof(out), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(out))
	TH_LOG("HBM pwrite: %s", strerror(errno));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0);
}

TEST_F(qdma, mm_round_trip_ddr)
{
	uint8_t out[512], in[512];

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0);

	fill_pattern(out, sizeof(out), 0x22);
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pwrite(self->qfd, out, sizeof(out), (off_t) SLASH_DDR_BASE),
		  (ssize_t) sizeof(out))
	TH_LOG("DDR pwrite: %s", strerror(errno));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in), (off_t) SLASH_DDR_BASE),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0);
}

/*
 * The revised masterplan contract: a qpair supports plain read()/write() at the
 * implicit file position, not only pread/pwrite.  llseek to the HBM window base,
 * stream the payload out with sequential write()s (each advancing the position),
 * then seek back and read it in with sequential read()s.  Round-trips the SAME
 * accelerator memory the pread/pwrite tests use.
 */
TEST_F(qdma, position_round_trip_hbm)
{
	uint8_t out[512], in[512];

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0);

	fill_pattern(out, sizeof(out), 0x33);
	memset(in, 0, sizeof(in));

	/* Stream the payload out in two halves with position-advancing write()s. */
	ASSERT_EQ(lseek(self->qfd, (off_t) SLASH_HBM_BASE, SEEK_SET),
		  (off_t) SLASH_HBM_BASE)
	TH_LOG("HBM lseek: %s", strerror(errno));
	ASSERT_EQ(write(self->qfd, out, 256), 256)
	TH_LOG("HBM write[0:256]: %s", strerror(errno));
	ASSERT_EQ(write(self->qfd, out + 256, 256), 256)
	TH_LOG("HBM write[256:512]: %s", strerror(errno));

	/* Read it back from the same base with sequential read()s. */
	ASSERT_EQ(lseek(self->qfd, (off_t) SLASH_HBM_BASE, SEEK_SET),
		  (off_t) SLASH_HBM_BASE);
	ASSERT_EQ(read(self->qfd, in, 256), 256)
	TH_LOG("HBM read[0:256]: %s", strerror(errno));
	ASSERT_EQ(read(self->qfd, in + 256, 256), 256)
	TH_LOG("HBM read[256:512]: %s", strerror(errno));

	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0);
}

/* Same position-based round-trip into the DDR window. */
TEST_F(qdma, position_round_trip_ddr)
{
	uint8_t out[512], in[512];

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0);

	fill_pattern(out, sizeof(out), 0x44);
	memset(in, 0, sizeof(in));

	ASSERT_EQ(lseek(self->qfd, (off_t) SLASH_DDR_BASE, SEEK_SET),
		  (off_t) SLASH_DDR_BASE)
	TH_LOG("DDR lseek: %s", strerror(errno));
	ASSERT_EQ(write(self->qfd, out, sizeof(out)), (ssize_t) sizeof(out))
	TH_LOG("DDR write: %s", strerror(errno));

	ASSERT_EQ(lseek(self->qfd, (off_t) SLASH_DDR_BASE, SEEK_SET),
		  (off_t) SLASH_DDR_BASE);
	ASSERT_EQ(read(self->qfd, in, sizeof(in)), (ssize_t) sizeof(in))
	TH_LOG("DDR read: %s", strerror(errno));

	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0);
}

/*
 * lseek SEEK_SET and SEEK_CUR reposition a qpair's implicit offset correctly.
 * (Per the masterplan, SEEK_END / st_size on a qpair are UNSPECIFIED -- the
 * qpair is an address window, not a sized file -- so we assert nothing about
 * them.)  SEEK_SET lands the next I/O at the chosen device address; SEEK_CUR is
 * relative to where the previous I/O advanced the position to.
 */
TEST_F(qdma, lseek_set_and_cur_reposition)
{
	uint8_t a[64], b[64], in[64];

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0);

	fill_pattern(a, sizeof(a), 0x61);
	fill_pattern(b, sizeof(b), 0x62);

	/* SEEK_SET to HBM base, write a -> lands at base; position now base+64. */
	ASSERT_EQ(lseek(self->qfd, (off_t) SLASH_HBM_BASE, SEEK_SET),
		  (off_t) SLASH_HBM_BASE);
	ASSERT_EQ(write(self->qfd, a, sizeof(a)), (ssize_t) sizeof(a));

	/* SEEK_CUR by +64 -> base+128; write b lands there. */
	ASSERT_EQ(lseek(self->qfd, 64, SEEK_CUR),
		  (off_t) (SLASH_HBM_BASE + 128));
	ASSERT_EQ(write(self->qfd, b, sizeof(b)), (ssize_t) sizeof(b));

	/* SEEK_CUR by 0 queries the current position (base+192). */
	ASSERT_EQ(lseek(self->qfd, 0, SEEK_CUR),
		  (off_t) (SLASH_HBM_BASE + 192));

	/* Verify the two regions hold a and b, and the 64-byte gap is untouched. */
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(a, in, sizeof(a)), 0);
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in),
			(off_t) (SLASH_HBM_BASE + 128)),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(b, in, sizeof(b)), 0);
}

TEST_F(qdma, st_mode_eopnotsupp)
{
	char path[4096];
	struct slash_abi_qdma_qpair_add req;
	int dfd;

	dev_path(path, sizeof(path), self->bdf, "qdma");
	dfd = open(path, O_RDONLY | O_DIRECTORY);
	ASSERT_GE(dfd, 0);

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);
	req.mode = 1; /* ST -- not supported */
	req.dir_mask = 0x3;
	EXPECT_EQ(ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), -1);
	EXPECT_EQ(errno, EOPNOTSUPP);
	close(dfd);
}

TEST_F(qdma, cmpt_dir_eopnotsupp)
{
	char path[4096];
	struct slash_abi_qdma_qpair_add req;
	int dfd;

	dev_path(path, sizeof(path), self->bdf, "qdma");
	dfd = open(path, O_RDONLY | O_DIRECTORY);
	ASSERT_GE(dfd, 0);

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);
	req.mode = 0;
	req.dir_mask = 0x4; /* CMPT -- not supported */
	EXPECT_EQ(ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), -1);
	EXPECT_EQ(errno, EOPNOTSUPP);
	close(dfd);
}

/*
 * QPAIR_ADD parameter validation (the [in] contract of the ioctl struct).  These
 * are distinct from the ST/CMPT -EOPNOTSUPP arms: a request that is MM with a
 * defined direction but otherwise malformed (no direction bit set, an undefined
 * direction bit, an out-of-range ring-size index, or an unknown mode) must be
 * rejected with -EINVAL -- not silently accepted (which would hand back a QID for
 * a nonsensical queue) and not conflated with -EOPNOTSUPP.  The ring-size fields
 * are CSR table indices 0..15 per the UAPI header; 16 is the first out-of-range
 * value.
 */
TEST_F(qdma, qpair_add_bad_params_einval)
{
	char path[4096];
	int dfd;

	dev_path(path, sizeof(path), self->bdf, "qdma");
	dfd = open(path, O_RDONLY | O_DIRECTORY);
	ASSERT_GE(dfd, 0);

	const struct {
		uint32_t mode;
		uint32_t dir_mask;
		uint32_t h2c, c2h, cmpt;
		const char *what;
	} bad[] = {
		{ 0, 0x0, 0, 0, 0, "no direction bit set" },
		{ 0, 0x8, 0, 0, 0, "undefined direction bit" },
		{ 0, 0x3, 16, 0, 0, "h2c ring index out of range" },
		{ 0, 0x3, 0, 16, 0, "c2h ring index out of range" },
		{ 2, 0x3, 0, 0, 0, "unknown mode 2" },
	};

	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		struct slash_abi_qdma_qpair_add req;

		memset(&req, 0, sizeof(req));
		req.size = sizeof(req);
		req.mode = bad[i].mode;
		req.dir_mask = bad[i].dir_mask;
		req.h2c_ring_sz = bad[i].h2c;
		req.c2h_ring_sz = bad[i].c2h;
		req.cmpt_ring_sz = bad[i].cmpt;
		errno = 0;
		EXPECT_EQ(ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), -1)
		TH_LOG("QPAIR_ADD should reject: %s", bad[i].what);
		EXPECT_EQ(errno, EINVAL)
		TH_LOG("QPAIR_ADD(%s) errno %d, expected EINVAL", bad[i].what,
		       errno);
	}
	close(dfd);
}

/*
 * The smallest valid QPAIR_ADD: MM, a single direction, all ring indices 0.
 * This is the positive counterpart to qpair_add_bad_params_einval -- it proves
 * the EINVAL gate above is not vacuously rejecting EVERYTHING (a daemon that
 * returned -EINVAL unconditionally would pass the negative test but fail here).
 */
TEST_F(qdma, qpair_add_minimal_ok)
{
	char path[4096];
	struct slash_abi_qdma_qpair_add req;
	int dfd, qfd;

	dev_path(path, sizeof(path), self->bdf, "qdma");
	dfd = open(path, O_RDONLY | O_DIRECTORY);
	ASSERT_GE(dfd, 0);

	memset(&req, 0, sizeof(req));
	req.size = sizeof(req);
	req.mode = 0;       /* MM */
	req.dir_mask = 0x1; /* H2C only -- a single direction is enough */
	ASSERT_EQ(ioctl(dfd, SLASH_ABI_QDMA_IOCTL_QPAIR_ADD, &req), 0)
	TH_LOG("minimal QPAIR_ADD (MM, H2C, rings=0): %s", strerror(errno));
	close(dfd);

	/* The qpair file appears and is openable. */
	snprintf(path + strlen(path), sizeof(path) - strlen(path), "/qpair%u",
		 req.qid);
	qfd = open(path, O_RDWR);
	EXPECT_GE(qfd, 0)
	TH_LOG("open qpair%u after minimal add: %s", req.qid, strerror(errno));
	if (qfd >= 0)
		close(qfd);
}

/* ──────────────────────────── unlinkability ───────────────────────────────── */
/*
 * Contract: only qpair<Q> files are unlinkable (the VRTD delete-on-last-close
 * pattern).  unlink of info / bar<M> / the global hotplug file -> -EPERM and the
 * file survives.
 */

FIXTURE(unlink_contract)
{
	char bdf[SLASH_PCI_BDF_LEN];
};

FIXTURE_SETUP(unlink_contract)
{
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
}

FIXTURE_TEARDOWN(unlink_contract) {}

TEST_F(unlink_contract, qpair_is_unlinkable)
{
	char path[4096];
	uint32_t qid = 0;
	int qfd;

	qfd = open_qpair(self->bdf, &qid);
	ASSERT_GE(qfd, 0);

	dev_path(path, sizeof(path), self->bdf, "qdma");
	snprintf(path + strlen(path), sizeof(path) - strlen(path), "/qpair%u",
		 qid);
	/* The VRTD pattern: unlink while open succeeds, and the fd stays usable. */
	EXPECT_EQ(unlink(path), 0)
	TH_LOG("unlink qpair%u: %s", qid, strerror(errno));

	uint8_t b[8] = { 1, 2, 3, 4, 5, 6, 7, 8 }, r[8] = { 0 };

	EXPECT_EQ(pwrite(qfd, b, sizeof(b), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(b));
	EXPECT_EQ(pread(qfd, r, sizeof(r), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(r));
	EXPECT_EQ(memcmp(b, r, sizeof(b)), 0);
	close(qfd);
}

TEST_F(unlink_contract, info_unlink_eperm_survives)
{
	char path[4096];

	dev_path(path, sizeof(path), self->bdf, "info");
	EXPECT_EQ(unlink(path), -1);
	EXPECT_EQ(errno, EPERM);
	EXPECT_EQ(access(path, F_OK), 0)
	TH_LOG("info must survive a rejected unlink");
}

TEST_F(unlink_contract, bar_unlink_eperm_survives)
{
	char path[4096];

	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	EXPECT_EQ(unlink(path), -1);
	EXPECT_EQ(errno, EPERM);
	EXPECT_EQ(access(path, F_OK), 0)
	TH_LOG("bar0 must survive a rejected unlink");
}

TEST_F(unlink_contract, hotplug_unlink_eperm_survives)
{
	char path[4096];

	snprintf(path, sizeof(path), "%s/hotplug", mount_root());
	EXPECT_EQ(unlink(path), -1);
	EXPECT_EQ(errno, EPERM);
	EXPECT_EQ(access(path, F_OK), 0)
	TH_LOG("hotplug must survive a rejected unlink");
}

/* ───────────────────────────── revocation ─────────────────────────────────── */
/*
 * The architecture's headline conformance requirement.  With an fd still OPEN on
 * an endpoint, REMOVE the owning function; then:
 *   - ops on the open fd -> -ENODEV,
 *   - reopen/lookup of the removed path -> -ENOENT,
 *   - close() always succeeds.
 * Covered for all three endpoint kinds: bar<M> (function .2), qpair<Q> (.1), and
 * info (whole-device revoke -- removing both functions tears the device down).
 *
 * Each revocation test runs in its OWN process (the harness forks per test) and
 * restores the device afterwards via RESCAN so later tests still find it.  Per
 * the architecture: function .1 == qdma, function .2 == bars.
 */

FIXTURE(revocation)
{
	char bdf[SLASH_PCI_BDF_LEN]; /* board-level, e.g. 0000:61:00 */
};

FIXTURE_SETUP(revocation)
{
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
}

/*
 * Restore the device (and any per-function-removed endpoints) for later tests.
 */
FIXTURE_TEARDOWN(revocation)
{
	char bf[64];

	bdf_func(bf, sizeof(bf), self->bdf, 1);
	(void) hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG, bf);
}

TEST_F(revocation, bar_fd_revoked_on_remove)
{
	char path[4096], bf[64];
	int fd;
	uint8_t buf[4] = { 0 };

	/* Open a BAR fd (function .2). */
	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	fd = open(path, O_RDWR);
	ASSERT_GE(fd, 0);

	/* REMOVE function 2 (bars). */
	bdf_func(bf, sizeof(bf), self->bdf, 2);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0)
	TH_LOG("REMOVE %s failed", bf);

	/* Op on the still-open fd -> -ENODEV. */
	errno = 0;
	EXPECT_EQ(pread(fd, buf, 4, 0), -1);
	EXPECT_EQ(errno, ENODEV);
	errno = 0;
	EXPECT_EQ(pwrite(fd, buf, 4, 0), -1);
	EXPECT_EQ(errno, ENODEV);

	/* Reopen of the removed path -> -ENOENT. */
	errno = 0;
	EXPECT_EQ(open(path, O_RDWR), -1);
	EXPECT_EQ(errno, ENOENT);

	/* close() always succeeds. */
	EXPECT_EQ(close(fd), 0);
}

TEST_F(revocation, qpair_fd_revoked_on_remove)
{
	char bf[64];
	uint32_t qid = 0;
	int qfd;
	uint8_t buf[8] = { 0 };

	/* Open a qpair fd (function .1). */
	qfd = open_qpair(self->bdf, &qid);
	ASSERT_GE(qfd, 0);

	/* REMOVE function 1 (qdma). */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0)
	TH_LOG("REMOVE %s failed", bf);

	/* Op on the still-open fd -> -ENODEV. */
	errno = 0;
	EXPECT_EQ(pwrite(qfd, buf, sizeof(buf), (off_t) SLASH_HBM_BASE), -1);
	EXPECT_EQ(errno, ENODEV);
	errno = 0;
	EXPECT_EQ(pread(qfd, buf, sizeof(buf), (off_t) SLASH_HBM_BASE), -1);
	EXPECT_EQ(errno, ENODEV);

	/* The qdma/ directory is gone: open of the qdma dir -> -ENOENT. */
	char qdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");
	errno = 0;
	EXPECT_EQ(open(qdir, O_RDONLY | O_DIRECTORY), -1);
	EXPECT_EQ(errno, ENOENT);

	/* close() always succeeds. */
	EXPECT_EQ(close(qfd), 0);
}

/*
 * info has no per-function owner: it lives at the device level and is revoked
 * only when the WHOLE device is torn down.  The only user-facing whole-device
 * teardown is HOTPLUG / TOGGLE_SBR, which remove-and-reload (the architecture's
 * "fully remove the accelerator, reload configuration, re-initialize").  So the
 * revocation we can observe for info is the in-flight handle going -ENODEV across
 * a whole-device HOTPLUG; the path then re-materialises (a fresh, working info),
 * which itself proves the OLD handle was revoked rather than silently reused.
 * (The reopen-> -ENOENT arm of the revocation contract is covered by the bar and
 * qpair tests, whose endpoints stay gone after a per-function REMOVE.)
 */
TEST_F(revocation, info_fd_revoked_on_whole_device_teardown)
{
	char path[4096], bf[64];
	int fd, fd2;
	struct slash_info in;

	/* Open the info fd. */
	dev_path(path, sizeof(path), self->bdf, "info");
	fd = open(path, O_RDONLY);
	ASSERT_GE(fd, 0);

	/* Whole-device HOTPLUG tears the device down (and re-inits it). */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG, bf), 0)
	TH_LOG("HOTPLUG %s failed", bf);

	/* Op on the still-open (now stale) info fd -> -ENODEV. */
	memset(&in, 0, sizeof(in));
	errno = 0;
	EXPECT_EQ(pread(fd, &in, sizeof(in), 0), -1);
	EXPECT_EQ(errno, ENODEV)
	TH_LOG("stale info fd op should be -ENODEV, errno=%d", errno);

	/* The device was re-initialised: a FRESH open works and reads a valid
	 * struct -- proving the old fd was a revoked handle, not the live one. */
	fd2 = open(path, O_RDONLY);
	ASSERT_GE(fd2, 0)
	TH_LOG("reopen after HOTPLUG re-init: %s", strerror(errno));
	EXPECT_EQ(read(fd2, &in, sizeof(in)), (ssize_t) sizeof(in));
	EXPECT_EQ(in.size, (uint32_t) sizeof(struct slash_info));

	/* close() always succeeds on the revoked handle. */
	EXPECT_EQ(close(fd), 0);
	close(fd2);
}

/* ───────────────────── hotplug semantics (sysemu) ──────────────────────────── */
/*
 * Contract: the 4 ioctls with sysemu semantics.  Per-function REMOVE (.1 removes
 * qdma, .2 removes bars, the other stays live).  BDF-with-function parsing errors
 * are rejected.  HOTPLUG/TOGGLE_SBR fully remove + re-init the device from config.
 * RESCAN reloads config and re-materialises a device whose WHOLE dir is gone
 * (it skips a BDF that still collides with a live accelerator, so it does NOT
 * re-add a single per-function-removed endpoint).  Each test restores a clean,
 * fully-present device in teardown via a whole-device HOTPLUG.
 */

FIXTURE(hotplug)
{
	char bdf[SLASH_PCI_BDF_LEN];
};

FIXTURE_SETUP(hotplug)
{
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());
}

FIXTURE_TEARDOWN(hotplug)
{
	char bf[64];

	/* HOTPLUG (whole-device remove + reload) restores everything from config,
	 * regardless of which function(s) a test removed. */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	(void) hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG, bf);
}

TEST_F(hotplug, per_function_remove_qdma_keeps_bars)
{
	char bf[64], qdir[4096], bdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");
	dev_path(bdir, sizeof(bdir), self->bdf, "bars");

	/* REMOVE .1 (qdma) -- bars must stay live. */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);

	EXPECT_EQ(access(qdir, F_OK), -1)
	TH_LOG("qdma/ should be gone after REMOVE .1");
	EXPECT_EQ(errno, ENOENT);
	EXPECT_EQ(access(bdir, F_OK), 0)
	TH_LOG("bars/ must survive REMOVE .1");
}

TEST_F(hotplug, per_function_remove_bars_keeps_qdma)
{
	char bf[64], qdir[4096], bdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");
	dev_path(bdir, sizeof(bdir), self->bdf, "bars");

	/* REMOVE .2 (bars) -- qdma must stay live. */
	bdf_func(bf, sizeof(bf), self->bdf, 2);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);

	EXPECT_EQ(access(bdir, F_OK), -1)
	TH_LOG("bars/ should be gone after REMOVE .2");
	EXPECT_EQ(errno, ENOENT);
	EXPECT_EQ(access(qdir, F_OK), 0)
	TH_LOG("qdma/ must survive REMOVE .2");
}

TEST_F(hotplug, remove_is_idempotent)
{
	char bf[64];

	bdf_func(bf, sizeof(bf), self->bdf, 1);
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);
	/* A second REMOVE of the same function is a no-op success. */
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);
}

TEST_F(hotplug, hotplug_ioctl_reinits_device)
{
	char bf[64], qdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");

	/* HOTPLUG = remove + reload + re-init: the device ends up present. */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG, bf), 0)
	TH_LOG("HOTPLUG %s failed", bf);
	EXPECT_EQ(access(qdir, F_OK), 0)
	TH_LOG("device should be re-initialised after HOTPLUG");
}

TEST_F(hotplug, toggle_sbr_reinits_device)
{
	char bf[64], qdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");

	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_TOGGLE_SBR, bf), 0)
	TH_LOG("TOGGLE_SBR %s failed", bf);
	EXPECT_EQ(access(qdir, F_OK), 0)
	TH_LOG("device should be re-initialised after TOGGLE_SBR");
}

/*
 * RESCAN rediscovers an individually-removed function of a still-live device:
 * after a per-function REMOVE the <BDF>/ dir survives (the sibling function is
 * still live), and RESCAN RESTORES the removed function rather than collision-
 * skipping it (the rediscovery contract -- "Rescans all PCI root buses to
 * discover new or reconfigured devices").  The surviving function stays live
 * throughout, and the restore is a REAL one (the rediscovered qdma can allocate
 * a qpair again), not just a re-created directory.
 *
 * A separate assertion pins that the rediscovery pass does NOT fabricate an
 * unconfigured/absent BDF: RESCAN restores the existing in-memory device, it does
 * not conjure a device for a BDF that was never configured.
 */
TEST_F(hotplug, rescan_rediscovers_removed_function)
{
	char bf[64], qdir[4096], bdir[4096];
	uint32_t qid = 0;
	int qfd;
	uint8_t pat[256], back[256];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");
	dev_path(bdir, sizeof(bdir), self->bdf, "bars");

	/*
	 * Before removing qdma, write a known pattern to HBM through a qpair.  After
	 * RESCAN rediscovers qdma we read the SAME HBM offset through a FRESH qpair
	 * and assert the bytes survived: that proves the rediscovered QDMA window is
	 * the SAME accelerator memory, not a freshly-zeroed store.
	 */
	qfd = open_qpair(self->bdf, &qid);
	ASSERT_GE(qfd, 0)
	TH_LOG("pre-remove open_qpair: %s", strerror(errno));
	fill_pattern(pat, sizeof(pat), 0x5c);
	ASSERT_EQ(pwrite(qfd, pat, sizeof(pat), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(pat))
	TH_LOG("pre-remove HBM pwrite: %s", strerror(errno));
	close(qfd);

	/* REMOVE .1 (qdma) -- bars stays live, qdma gone. */
	bdf_func(bf, sizeof(bf), self->bdf, 1);
	ASSERT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);
	ASSERT_EQ(access(qdir, F_OK), -1);
	ASSERT_EQ(access(bdir, F_OK), 0);

	/* RESCAN rediscovers the removed function: qdma is restored, bars stays. */
	ASSERT_EQ(hotplug_rescan(), 0);
	EXPECT_EQ(access(qdir, F_OK), 0)
	TH_LOG("RESCAN should rediscover the per-function-removed qdma endpoint");
	EXPECT_EQ(access(bdir, F_OK), 0)
	TH_LOG("the surviving function must remain live across RESCAN");

	/* The restore is real: the rediscovered qdma can allocate a qpair again. */
	qfd = open_qpair(self->bdf, &qid);
	EXPECT_GE(qfd, 0)
	TH_LOG("rediscovered qdma must be usable (QPAIR_ADD): %s",
	       strerror(errno));
	if (qfd >= 0) {
		/* The accelerator memory survived the per-function remove + rescan:
		 * a NEW qpair sees the bytes the OLD qpair wrote before removal. */
		memset(back, 0, sizeof(back));
		EXPECT_EQ(pread(qfd, back, sizeof(back), (off_t) SLASH_HBM_BASE),
			  (ssize_t) sizeof(back))
		TH_LOG("post-rescan HBM pread: %s", strerror(errno));
		EXPECT_EQ(memcmp(pat, back, sizeof(pat)), 0)
		TH_LOG("rediscovered QDMA must hit the SAME accelerator memory");
		close(qfd);
	}

	/* RESCAN must NOT fabricate a device for a BDF that was never configured:
	 * a never-present BDF directory stays absent after a rescan. */
	char absent[4096];

	snprintf(absent, sizeof(absent), "%s/0000:de:00", mount_root());
	ASSERT_EQ(hotplug_rescan(), 0);
	EXPECT_EQ(access(absent, F_OK), -1)
	TH_LOG("RESCAN must not fabricate an unconfigured BDF");
	EXPECT_EQ(errno, ENOENT);
}

TEST_F(hotplug, malformed_bdf_rejected)
{
	/* A BDF without a function component is not a valid device request. */
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, "not-a-bdf"),
		  -EINVAL);
	/* A two-digit function is malformed. */
	char bf[64];

	snprintf(bf, sizeof(bf), "%s.12", self->bdf);
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf),
		  -EINVAL);
	/* A trailing dot with no function digit is malformed. */
	snprintf(bf, sizeof(bf), "%s.", self->bdf);
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf),
		  -EINVAL);
	/* A non-digit function suffix is malformed. */
	snprintf(bf, sizeof(bf), "%s.x", self->bdf);
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf),
		  -EINVAL);
}

/*
 * The real accelerator as a physical function 0, which the daemon doesn't
 * emulate. Still, a user may try to remove it, which the daemon should expect.
 * The expected behavior is a no-op.
 */
TEST_F(hotplug, function_0_noop)
{
	char bf[64], qdir[4096], bdir[4096];

	dev_path(qdir, sizeof(qdir), self->bdf, "qdma");
	dev_path(bdir, sizeof(bdir), self->bdf, "bars");

	bdf_func(bf, sizeof(bf), self->bdf, 0);
	EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf), 0);

	ASSERT_EQ(access(bdir, F_OK), 0);
	ASSERT_EQ(access(qdir, F_OK), 0);
}

/*
 * A SYNTACTICALLY valid BDF.F whose function digit is well-formed but names a
 * function that is not a removable endpoint (only .1 == qdma and .2 == bars are
 * emulated) is rejected with -EOPNOTSUPP, distinctly from the -EINVAL the
 * malformed-syntax cases get.  This pins that the daemon distinguishes "I cannot
 * parse this" (-EINVAL) from "I parsed it but that function is not removable"
 * (-EOPNOTSUPP) -- a teeth distinction the malformed_bdf_rejected test alone does
 * not exercise.
 */
TEST_F(hotplug, unremovable_function_eopnotsupp)
{
	char bf[64];

	for (size_t i = 3; i < 8; i++) {
		bdf_func(bf, sizeof(bf), self->bdf, i);
		EXPECT_EQ(hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_REMOVE, bf),
			-EOPNOTSUPP)
		TH_LOG("REMOVE of function .%lu must be -EOPNOTSUPP (no such function)", i);
	}
}

/* ──────────────────────────── reconfiguration ─────────────────────────────── */
/*
 * Contract: a SINGLE write of a whole (stub) VBIN to the reconfiguration region
 * through an opened qpair triggers reconfiguration (the daemon reassembles
 * kernel-split chunks).  A reconfig-region READ -> -ERANGE.  A malformed VBIN
 * fails the write (-EINVAL) WITHOUT wedging the daemon (a subsequent ordinary
 * transfer still works).  After reconfig, a BAR/QDMA op round-trips through the
 * model.
 *
 * These tests require a model executable to pack as "vpp_sim"
 * (SLASH_CONFORMANCE_STUB_MODEL); without it (e.g. a kernel backend) they SKIP.
 */

FIXTURE(reconfig)
{
	char bdf[SLASH_PCI_BDF_LEN];
	int qfd;
	uint32_t qid;
	uint8_t *vbin;
	size_t vbin_len;
};

FIXTURE_SETUP(reconfig)
{
	const char *model = stub_model_path();

	self->qfd = -1;
	self->vbin = NULL;
	self->vbin_len = 0;

	if (model == NULL)
		SKIP(return, "SLASH_CONFORMANCE_STUB_MODEL unset (no VBIN backend)");
	if (discover_bdf(self->bdf, sizeof(self->bdf)) != 0)
		SKIP(return, "no device directory under %s", mount_root());

	self->vbin = make_ci_vbin(model, &self->vbin_len);
	ASSERT_NE(self->vbin, NULL)
	TH_LOG("failed to build CI VBIN from %s", model);

	self->qfd = open_qpair(self->bdf, &self->qid);
	ASSERT_GE(self->qfd, 0)
	TH_LOG("open_qpair: %s", strerror(errno));
}

FIXTURE_TEARDOWN(reconfig)
{
	if (self->qfd >= 0)
		close(self->qfd);
	free(self->vbin);
	/* A reconfiguration leaves a model running on the device.  Restore a clean,
	 * fully-present device for later tests with a whole-device HOTPLUG (remove +
	 * reload), which also tears the model down and re-materialises every
	 * endpoint from config. */
	if (self->bdf[0] != '\0') {
		char bf[64];

		bdf_func(bf, sizeof(bf), self->bdf, 1);
		(void) hotplug_dev_ioctl(SLASH_ABI_HOTPLUG_IOCTL_HOTPLUG, bf);
	}
}

TEST_F(reconfig, single_write_vbin_then_data_plane_round_trips)
{
	char path[4096];
	int bfd;
	uint32_t regval = 0x1234abcdu, readback = 0;
	uint8_t out[1024], in[1024];

	/* One write of the whole VBIN to the reconfig region spawns the model. */
	ASSERT_EQ(pwrite(self->qfd, self->vbin, self->vbin_len,
			 (off_t) SLASH_RECONFIG_BASE),
		  (ssize_t) self->vbin_len)
	TH_LOG("reconfig write: %s", strerror(errno));

	/* BAR0 register round-trips THROUGH the model. */
	bar_path(path, sizeof(path), self->bdf, SLASH_BAR_USER_IDX);
	bfd = open(path, O_RDWR);
	ASSERT_GE(bfd, 0);
	ASSERT_EQ(pwrite(bfd, &regval, 4, 0x40), 4)
	TH_LOG("bar write: %s", strerror(errno));
	ASSERT_EQ(pread(bfd, &readback, 4, 0x40), 4);
	EXPECT_EQ(readback, regval);
	close(bfd);

	/* QDMA MM round-trips THROUGH the model (DDR). */
	fill_pattern(out, sizeof(out), 0x5a);
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pwrite(self->qfd, out, sizeof(out), (off_t) SLASH_DDR_BASE),
		  (ssize_t) sizeof(out))
	TH_LOG("qpair write through model: %s", strerror(errno));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in), (off_t) SLASH_DDR_BASE),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0);
}

/*
 * The T10 chunk-reassembly contract (a real, fixed bug): a SINGLE pwrite(2) of a
 * VBIN larger than the FUSE max_write is split by the kernel into several
 * contiguous in-region writes.  The daemon must reassemble those chunks into the
 * one whole VBIN and spawn the model from it.  The small-VBIN test above fits in
 * a single FUSE write and therefore never exercises this path; here the payload
 * is deliberately several MiB so the kernel MUST fragment it.  Success is proven
 * end-to-end: after the one large write, the data plane round-trips THROUGH the
 * reassembled model -- a daemon that mishandled reassembly would either fail the
 * write or fail to bring the model up, and the round-trip would not hold.
 *
 * The harness's per-test alarm() bounds this so a reassembly stall FAILS rather
 * than hangs.
 */
TEST_F(reconfig, large_multichunk_vbin_reassembles_and_round_trips)
{
	uint8_t *big;
	size_t big_len = 0;
	uint8_t out[1024], in[1024];

	/* >= 2 MiB forces fragmentation under any FUSE max_write (default is at
	 * most ~1 MiB). */
	big = make_large_vbin(stub_model_path(), 2u * 1024u * 1024u, &big_len);
	ASSERT_NE(big, NULL)
	TH_LOG("failed to build large VBIN from %s", stub_model_path());
	ASSERT_GT(big_len, 2u * 1024u * 1024u)
	TH_LOG("large VBIN only %zu bytes -- would not fragment", big_len);

	/* One pwrite of the whole multi-MiB VBIN; the kernel splits it. */
	ssize_t w = pwrite(self->qfd, big, big_len, (off_t) SLASH_RECONFIG_BASE);
	int werr = errno;

	free(big);
	ASSERT_EQ(w, (ssize_t) big_len)
	TH_LOG("large reconfig write returned %zd (%s)", w, strerror(werr));

	/* The reassembled model is live: a QDMA MM transfer round-trips through it. */
	fill_pattern(out, sizeof(out), 0x77);
	memset(in, 0, sizeof(in));
	ASSERT_EQ(pwrite(self->qfd, out, sizeof(out), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(out))
	TH_LOG("post-reassembly write: %s", strerror(errno));
	ASSERT_EQ(pread(self->qfd, in, sizeof(in), (off_t) SLASH_HBM_BASE),
		  (ssize_t) sizeof(in));
	EXPECT_EQ(memcmp(out, in, sizeof(out)), 0)
	TH_LOG("data plane did not round-trip through the reassembled model");
}

TEST_F(reconfig, region_read_is_erange)
{
	uint8_t buf[8];

	EXPECT_EQ(pread(self->qfd, buf, sizeof(buf),
			(off_t) SLASH_RECONFIG_BASE),
		  -1);
	EXPECT_EQ(errno, ERANGE);
}

/*
 * A non-contiguous write into the reconfiguration region is not how a VBIN is
 * delivered (a VBIN is one write, which the kernel may split into CONTIGUOUS
 * chunks).  A write that seeks into the middle of the region without a transfer
 * in progress at that offset must be rejected (-EINVAL) rather than silently
 * accepted into the reassembly buffer where it would corrupt a later VBIN.  After
 * the rejection the daemon must stay serviceable AND a fresh full VBIN at BASE
 * must still spawn -- i.e. the bogus seek did not leave the reassembler wedged in
 * a partial state.  This exercises the reassembler's contiguity gate, which the
 * single-shot and large-VBIN tests (both of which start at BASE) do not.
 */
TEST_F(reconfig, noncontiguous_region_write_rejected_then_recovers)
{
	uint8_t junk[512];
	uint8_t byte = 0x5e, back = 0;

	/* A structurally-plausible-sized write that SEEKS past BASE with no
	 * transfer in progress: the reassembler expects either BASE (start) or
	 * BASE+acc_len (append), so this is neither. */
	memset(junk, 0x42, sizeof(junk));
	EXPECT_EQ(pwrite(self->qfd, junk, sizeof(junk),
			 (off_t) (SLASH_RECONFIG_BASE + 4096)),
		  -1)
	TH_LOG("seeking reconfig write should be rejected");
	EXPECT_EQ(errno, EINVAL);

	/* The daemon stays serviceable: an ordinary HBM transfer still works. */
	EXPECT_EQ(pwrite(self->qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1)
	TH_LOG("daemon wedged after non-contiguous reconfig: %s",
	       strerror(errno));
	EXPECT_EQ(pread(self->qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
	EXPECT_EQ(back, byte);

	/* Recovery: a fresh full VBIN at BASE still spawns and round-trips. */
	ASSERT_EQ(pwrite(self->qfd, self->vbin, self->vbin_len,
			 (off_t) SLASH_RECONFIG_BASE),
		  (ssize_t) self->vbin_len)
	TH_LOG("recovery reconfig write after bad seek: %s", strerror(errno));
	
	EXPECT_EQ(pwrite(self->qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1)
	TH_LOG("daemon wedged after non-contiguous reconfig: %s",
	       strerror(errno));
	EXPECT_EQ(pread(self->qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
	EXPECT_EQ(back, byte);
}

TEST_F(reconfig, malformed_vbin_fails_without_wedging)
{
	uint8_t junk[512];
	uint8_t byte = 0x99, back = 0;

	/* A full non-zero block lacking the ustar magic is structurally INVALID;
	 * the write fails up front and no model is spawned. */
	memset(junk, 0xAB, sizeof(junk));
	EXPECT_EQ(pwrite(self->qfd, junk, sizeof(junk),
			 (off_t) SLASH_RECONFIG_BASE),
		  -1);
	EXPECT_EQ(errno, EINVAL);

	/* The daemon stays serviceable: an ordinary HBM transfer still works. */
	EXPECT_EQ(pwrite(self->qfd, &byte, 1, (off_t) SLASH_HBM_BASE), 1)
	TH_LOG("daemon wedged after bad VBIN: %s", strerror(errno));
	EXPECT_EQ(pread(self->qfd, &back, 1, (off_t) SLASH_HBM_BASE), 1);
	EXPECT_EQ(back, byte);

	/* Recovery: a fresh full VBIN at BASE still spawns and round-trips. */
	ASSERT_EQ(pwrite(self->qfd, self->vbin, self->vbin_len,
			 (off_t) SLASH_RECONFIG_BASE),
		  (ssize_t) self->vbin_len)
	TH_LOG("recovery reconfig write: %s", strerror(errno));
}

TEST_HARNESS_MAIN
