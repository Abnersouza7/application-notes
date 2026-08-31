---
doc_id: ANXXXX
title: <Title in sentence case, naming the device and the task>
rev: 1.0
date: <D Month YYYY>
kind: Application note
company: Binho Inc.
website: binho.io
keywords: ANXXXX, <product>, <bus>, <part numbers>, <task>
trademarks: |
  <Only the marks this note actually references. The shared legal page supplies the
  generic sentence and Binho's own marks; this field supplies the rest. Claiming a
  vendor's marks in a note that does not mention them is worse than claiming none,
  so check the note text before filling this in. The build refuses without it.>
abstract: |
  Three to five sentences. What the document enables, on which hardware, and what is
  supplied with it. Written for someone deciding whether this note answers their question.
---

## 1 Introduction

Why this exists and when it applies. State the problem in the reader's terms before naming
any Binho product.

### 1.1 Scope

What the document covers, as a list, and one short paragraph naming what it does not cover.

### 1.2 Applicable devices

A table of parts, with the basis for each row. Where a row was not tested, say so.

Table: Applicable devices

| Family | Example parts | Identifier | Basis |
|---|---|---|---|
| | | | |

### 1.3 Related documents

Table: Related documents

| Document | Title |
|---|---|
| | |

## 2 System overview

The mental model the reader needs before the procedure. Roles on the bus, who initiates
what, and any mechanism that differs from what an experienced reader would assume. If there
is one non-obvious design fact that shapes everything else, it belongs here.

<!-- Every table needs a 'Table: <caption>' line above it. The build numbers
     them in document order; refer to them by number in the prose. -->

## 3 Hardware setup

### 3.1 Required equipment

### 3.2 Connections

A table of connections, marking which are required and which are optional.

Table: Connections between the Supernova and the target

| Supernova | Target | Required |
|---|---|---|
| | | |

### 3.3 Bus considerations

Voltage, termination, pull-ups, anything that damages hardware if wrong. Use a Caution
callout for those.

## 4 Software setup

Install steps, exact package versions used.

## 5 <Protocol or mechanism>

The reference section. Framing, registers, commands, timing. Enough that a reader could
write their own host without this document's code.

## 6 Procedure

Numbered, in the order performed, each step with the command that performs it and the output
it produces. Show real output, not invented output.

## 7 Reference

Command and option tables for any supplied utility.

## 8 Measured results

### 8.1 Test configuration

Every version number needed to reproduce the measurement.

### 8.2 Results

State what was measured and what limits it. Do not present a measurement as a specification.

## 9 Troubleshooting

Table: Troubleshooting

| Symptom | Probable cause and action |
|---|---|
| | |

## 10 Acronyms

Table: Acronyms

| Term | Meaning |
|---|---|
| | |

## 11 Note about the source code in the document

Example code shown in this document and supplied in the accompanying archive is provided for
evaluation and reference. It is released under the MIT license, the text of which is included
in the archive.

The archive `ANXXXX-assets.zip` contains <list the contents>.

## 12 Revision history

Table: Revision history

| Revision | Date | Description |
|---|---|---|
| 1.0 | <date> | Initial release. |
