// Native CLI: runs the same JSON requests as the WASM build, for cloud jobs and
// for benchmarking against the WASM build.
//
//   cadoc-kernel-cli REQUESTS.jsonl [--iterations N] [--out DIR]
//   cadoc-kernel-cli --stdio
//   cadoc-kernel-cli --version
//
// --stdio is the native kernel's protocol transport, the same wire format as the reference
// kernel (kernel/reference/README.md): one Envelope<KernelRequest> per stdin line, one
// Envelope<KernelResult | KernelError> per stdout line, in order, until EOF. Typed arrays are
// base64, non-finite numbers {"$float": ...}. stdout carries protocol lines only: anything
// OCCT prints goes to stderr. pipeline/kernel/native-client.ts speaks it.
//
// Each line of REQUESTS.jsonl is a protocol request. An "import" line carries an
// extra "path" field: the file is read into memory BEFORE the timed call, so the
// kernel sees bytes exactly as it does in the browser. Each iteration replays all
// lines; per-call timings (measured in C++) go to stdout as JSON lines. With
// --out, the responses of the last iteration are written there.
// SPDX-License-Identifier: LGPL-2.1-only
#include "kernel.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) { std::cerr << "cannot read " << p << "\n"; std::exit(2); }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Extracts "path":"..." from an import line (paths are plain, no escapes).
static std::string pathOf(const std::string& line) {
  auto k = line.find("\"path\"");
  if (k == std::string::npos) return "";
  auto a = line.find('"', line.find(':', k) + 1);
  auto b = line.find('"', a + 1);
  return line.substr(a + 1, b - a - 1);
}

static int stdioTransport() {
  // Keep a private copy of the real stdout for the protocol; point fd 1 at stderr.
  FILE* proto = fdopen(dup(1), "w");
  dup2(2, 1);
  std::ios::sync_with_stdio(false);
  for (std::string line; std::getline(std::cin, line);) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    const std::string out = cadoc::handleEnvelopeLine(line);
    std::fwrite(out.data(), 1, out.size(), proto);
    std::fputc('\n', proto);
    std::fflush(proto);
  }
  return 0;
}

int main(int argc, char** argv) {
  if (argc == 2 && !std::strcmp(argv[1], "--stdio")) return stdioTransport();
  if (argc == 2 && !std::strcmp(argv[1], "--version")) {
    std::printf("%s\n", cadoc::version().c_str());
    return 0;
  }
  if (argc < 2) { std::cerr << "usage: cadoc-kernel-cli REQUESTS.jsonl [--iterations N] [--out DIR]\n"; return 2; }
  int iterations = 1;
  std::string outDir;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--iterations") && i + 1 < argc) iterations = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) outDir = argv[++i];
  }
  std::vector<std::string> lines;
  {
    std::istringstream in(slurp(argv[1]));
    for (std::string l; std::getline(in, l);) if (!l.empty()) lines.push_back(l);
  }
  std::vector<std::string> payloads;
  for (auto& l : lines) {
    std::string p = pathOf(l);
    payloads.push_back(p.empty() ? std::string() : slurp(p));
  }
  for (int it = 0; it < iterations; ++it) {
    for (size_t i = 0; i < lines.size(); ++i) {
      const std::string& b = payloads[i];
      std::string resp = cadoc::handle(lines[i], reinterpret_cast<const uint8_t*>(b.data()), b.size());
      struct rusage ru;
      getrusage(RUSAGE_SELF, &ru);
      std::printf("{\"iteration\":%d,\"request\":%zu,\"maxRssKiB\":%ld,\"timing\":%s}\n", it, i, ru.ru_maxrss,
                  cadoc::lastTiming().c_str());
      std::fflush(stdout);
      if (resp.rfind("{\"type\":\"error\"", 0) == 0) std::fprintf(stderr, "request %zu: %s\n", i, resp.c_str());
      if (!outDir.empty() && it == iterations - 1) {
        std::ofstream(outDir + "/response-" + std::to_string(i) + ".json", std::ios::binary) << resp;
      }
    }
  }
  return 0;
}
