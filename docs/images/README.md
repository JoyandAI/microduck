# Images

The `.svg` files here are **placeholders**. Each one renders as a dashed grey box naming the shot it
is standing in for, so the README lays out correctly before any photo exists.

To fill one in: drop a `.jpg`, `.png` or `.gif` next to it, point the README's `<img src>` at the new
file, and delete the placeholder. Keep the sizes — the layout is built around them.

| placeholder | what goes there | size |
|---|---|---|
| `hero.svg` | The robot, three-quarter view, in room light. The one image somebody sees first. | 1200×500 |
| `walk.svg` | A lap of the desk, side on. Short loop, no audio. | 560×340 |
| `roller.svg` | Wheels on, rolling past the camera. | 560×340 |
| `ground-pick.svg` | Beak to the floor, picking something up. | 560×340 |
| `standup.svg` | Face-down to standing, in one take. | 560×340 |

GitHub serves these through its own proxy, so a `.gif` over a couple of megabytes is slow on the
landing page — keep loops short and the palette small.

## Videos

A real video is better than a GIF for anything longer than a second or two, and GitHub hosts it for
you. Drop the file into the comment box of any issue or pull request (or the web editor's) — do not
commit it — and GitHub uploads it and hands back a URL like:

```
https://github.com/user-attachments/assets/8d7a530b-37f0-4d76-b47e-02cce6b326cb
```

**Where that URL goes depends on where in the page you are**, and this is the part that wastes an
afternoon:

- **In ordinary markdown**, the bare URL on a line of its own becomes a player. Nothing else needed.
- **Inside HTML** — which the "It does things" table is — markdown is not parsed at all, so a bare
  URL stays plain text and a link stays a link. Use the element instead:

  ```html
  <video src="https://github.com/user-attachments/assets/8d7a530b-…" controls muted loop width="100%"></video>
  ```

  `controls`, `muted` and `loop` are the attributes worth relying on; treat `autoplay` as something
  that may not survive GitHub's sanitiser, so assume a viewer presses play.

Each cell in the README's table carries that line commented out beside its placeholder, so filling
one in is: upload, paste the id, delete the `<img>`.

Two consequences of GitHub hosting the file rather than the repo. It is **not in a clone** — the
README's videos are blank without a network, and a mirror of this repo still fetches them from
github.com. And GitHub caps attachment size, so a long clip belongs on the
[website](https://pollen-robotics.com/microduck) with a short one here.
