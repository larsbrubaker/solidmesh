//! Clipper 6's horizontal edges and scanbeam tops (`clipper.cs`): `ProcessHorizontal(s)`, the
//! maxima list `StrictlySimple` uses, `ProcessEdgesAtTopOfScanbeam` (maxima, promoting
//! intermediate vertices and horizontals), `DoMaxima` and the maxima-pair lookups.

use super::clipper_base::{is_horizontal, slopes_equal4, slopes_equal_edges};
use super::engine::{horz_segments_overlap, top_x, Clipper};
use super::types::{ip, ClipperError, Direction, Maxima, TEdge, SKIP, UNASSIGNED};

/// C# `GetHorzDirection`: (direction, left, right).
fn get_horz_direction(horz_edge: &TEdge) -> (Direction, i64, i64) {
    if horz_edge.bot.x < horz_edge.top.x {
        (Direction::LeftToRight, horz_edge.bot.x, horz_edge.top.x)
    } else {
        (Direction::RightToLeft, horz_edge.top.x, horz_edge.bot.x)
    }
}

impl Clipper {
    /// C# `InsertMaxima`: double-linked list sorted ascending, ignoring dups.
    fn insert_maxima(&mut self, x: i64) {
        let new_index = self.maxima.len();
        match self.maxima_list {
            None => {
                self.maxima.push(Maxima {
                    x,
                    next: None,
                    prev: None,
                });
                self.maxima_list = Some(new_index);
            }
            Some(head) if x < self.maxima[head].x => {
                self.maxima.push(Maxima {
                    x,
                    next: Some(head),
                    prev: None,
                });
                // The C# never repoints the old head's Prev here; neither does this.
                self.maxima_list = Some(new_index);
            }
            Some(head) => {
                let mut m = head;
                while let Some(next) = self.maxima[m].next {
                    if x >= self.maxima[next].x {
                        m = next;
                    } else {
                        break;
                    }
                }
                if x == self.maxima[m].x {
                    return; //ie ignores duplicates
                }
                //insert newMax between m and m.Next ...
                let next = self.maxima[m].next;
                self.maxima.push(Maxima {
                    x,
                    next,
                    prev: Some(m),
                });
                if let Some(next) = next {
                    self.maxima[next].prev = Some(new_index);
                }
                self.maxima[m].next = Some(new_index);
            }
        }
    }

    /// C# `ProcessHorizontals`.
    pub(super) fn process_horizontals(&mut self) -> Result<(), ClipperError> {
        while let Some(horz_edge) = self.pop_edge_from_sel() {
            self.process_horizontal(horz_edge)?;
        }
        Ok(())
    }

    /// The edges of the SEL that overlap `horz_edge` get a join to `op1` (C# loop shared by
    /// two places in `ProcessHorizontal`).
    fn join_overlapping_horizontals(&mut self, horz_edge: usize, op1: usize) {
        let mut e_next_horz = self.sorted_edges;
        while let Some(next_horz) = e_next_horz {
            if self.edges[next_horz].out_idx >= 0
                && horz_segments_overlap(
                    self.edges[horz_edge].bot.x,
                    self.edges[horz_edge].top.x,
                    self.edges[next_horz].bot.x,
                    self.edges[next_horz].top.x,
                )
            {
                let op2 = self.get_last_out_pt(next_horz);
                self.add_join(op2, op1, self.edges[next_horz].top);
            }
            e_next_horz = self.edges[next_horz].next_in_sel;
        }
    }

    fn process_horizontal(&mut self, mut horz_edge: usize) -> Result<(), ClipperError> {
        let is_open = self.edges[horz_edge].wind_delta == 0;

        let (mut dir, mut horz_left, mut horz_right) = get_horz_direction(&self.edges[horz_edge]);

        let mut e_last_horz = horz_edge;
        let mut e_max_pair = None;
        while let Some(next) = self.edges[e_last_horz].next_in_lml {
            if !is_horizontal(&self.edges[next]) {
                break;
            }
            e_last_horz = next;
        }
        if self.edges[e_last_horz].next_in_lml.is_none() {
            e_max_pair = self.get_maxima_pair(e_last_horz);
        }

        let mut curr_max = self.maxima_list;
        if let Some(mut m) = curr_max {
            //get the first maxima in range (X) ...
            if dir == Direction::LeftToRight {
                let mut cm = Some(m);
                while let Some(c) = cm {
                    if self.maxima[c].x <= self.edges[horz_edge].bot.x {
                        cm = self.maxima[c].next;
                    } else {
                        break;
                    }
                }
                if let Some(c) = cm {
                    if self.maxima[c].x >= self.edges[e_last_horz].top.x {
                        cm = None;
                    }
                }
                curr_max = cm;
            } else {
                while let Some(next) = self.maxima[m].next {
                    if self.maxima[next].x < self.edges[horz_edge].bot.x {
                        m = next;
                    } else {
                        break;
                    }
                }
                curr_max = if self.maxima[m].x <= self.edges[e_last_horz].top.x {
                    None
                } else {
                    Some(m)
                };
            }
        }

        let mut op1: Option<usize> = None;
        loop {
            //loop through consec. horizontal edges
            let is_last_horz = horz_edge == e_last_horz;
            let mut e = self.get_next_in_ael(horz_edge, dir);
            while let Some(edge) = e {
                //this code block inserts extra coords into horizontal edges (in output
                //polygons) whereever maxima touch these horizontal edges. This helps
                //'simplifying' polygons (ie if the Simplify property is set).
                if curr_max.is_some() {
                    if dir == Direction::LeftToRight {
                        while let Some(c) = curr_max {
                            if self.maxima[c].x >= self.edges[edge].curr.x {
                                break;
                            }
                            if self.edges[horz_edge].out_idx >= 0 && !is_open {
                                let pt = ip(self.maxima[c].x, self.edges[horz_edge].bot.y);
                                self.add_out_pt(horz_edge, pt);
                            }
                            curr_max = self.maxima[c].next;
                        }
                    } else {
                        while let Some(c) = curr_max {
                            if self.maxima[c].x <= self.edges[edge].curr.x {
                                break;
                            }
                            if self.edges[horz_edge].out_idx >= 0 && !is_open {
                                let pt = ip(self.maxima[c].x, self.edges[horz_edge].bot.y);
                                self.add_out_pt(horz_edge, pt);
                            }
                            curr_max = self.maxima[c].prev;
                        }
                    }
                }

                let e_curr_x = self.edges[edge].curr.x;
                if (dir == Direction::LeftToRight && e_curr_x > horz_right)
                    || (dir == Direction::RightToLeft && e_curr_x < horz_left)
                {
                    break;
                }

                //Also break if we've got to the end of an intermediate horizontal edge ...
                //nb: Smaller Dx's are to the right of larger Dx's ABOVE the horizontal.
                if e_curr_x == self.edges[horz_edge].top.x {
                    if let Some(next_lml) = self.edges[horz_edge].next_in_lml {
                        if self.edges[edge].dx < self.edges[next_lml].dx {
                            break;
                        }
                    }
                }

                if self.edges[horz_edge].out_idx >= 0 && !is_open {
                    //note: may be done multiple times
                    let op = self.add_out_pt(horz_edge, self.edges[edge].curr);
                    op1 = Some(op);
                    self.join_overlapping_horizontals(horz_edge, op);
                    self.add_ghost_join(op, self.edges[horz_edge].bot);
                }

                //OK, so far we're still in range of the horizontal Edge  but make sure
                //we're at the last of consec. horizontals when matching with eMaxPair
                if Some(edge) == e_max_pair && is_last_horz {
                    if self.edges[horz_edge].out_idx >= 0 {
                        let top = self.edges[horz_edge].top;
                        self.add_local_max_poly(horz_edge, edge, top);
                    }
                    self.delete_from_ael(horz_edge);
                    self.delete_from_ael(edge);
                    return Ok(());
                }

                let pt = ip(self.edges[edge].curr.x, self.edges[horz_edge].curr.y);
                if dir == Direction::LeftToRight {
                    self.intersect_edges(horz_edge, edge, pt);
                } else {
                    self.intersect_edges(edge, horz_edge, pt);
                }
                let e_next = self.get_next_in_ael(edge, dir);
                self.swap_positions_in_ael(horz_edge, edge);
                e = e_next;
            } //end while(e != null)

            //Break out of loop if HorzEdge.NextInLML is not also horizontal ...
            match self.edges[horz_edge].next_in_lml {
                Some(next) if is_horizontal(&self.edges[next]) => {}
                _ => break,
            }

            horz_edge = self.update_edge_into_ael(horz_edge)?;
            if self.edges[horz_edge].out_idx >= 0 {
                self.add_out_pt(horz_edge, self.edges[horz_edge].bot);
            }
            (dir, horz_left, horz_right) = get_horz_direction(&self.edges[horz_edge]);
        } //end for (;;)

        if self.edges[horz_edge].out_idx >= 0 && op1.is_none() {
            let op = self.get_last_out_pt(horz_edge);
            self.join_overlapping_horizontals(horz_edge, op);
            self.add_ghost_join(op, self.edges[horz_edge].top);
        }

        if self.edges[horz_edge].next_in_lml.is_some() {
            if self.edges[horz_edge].out_idx >= 0 {
                let op = self.add_out_pt(horz_edge, self.edges[horz_edge].top);

                horz_edge = self.update_edge_into_ael(horz_edge)?;
                if self.edges[horz_edge].wind_delta == 0 {
                    return Ok(());
                }
                //nb: HorzEdge is no longer horizontal here
                let e_prev = self.edges[horz_edge].prev_in_ael;
                let e_next = self.edges[horz_edge].next_in_ael;
                let h = self.edges[horz_edge].clone();
                if let Some(prev) = e_prev.filter(|&p| self.shares_bottom(&h, &self.edges[p])) {
                    let op2 = self.add_out_pt(prev, h.bot);
                    self.add_join(op, op2, h.top);
                } else if let Some(next) =
                    e_next.filter(|&n| self.shares_bottom(&h, &self.edges[n]))
                {
                    let op2 = self.add_out_pt(next, h.bot);
                    self.add_join(op, op2, h.top);
                }
            } else {
                self.update_edge_into_ael(horz_edge)?;
            }
        } else {
            if self.edges[horz_edge].out_idx >= 0 {
                self.add_out_pt(horz_edge, self.edges[horz_edge].top);
            }
            self.delete_from_ael(horz_edge);
        }
        Ok(())
    }

    /// The condition `ProcessHorizontal` checks on the edge either side of a horizontal that
    /// has just turned into a sloped edge: it is a contributing closed edge starting at the
    /// same bottom with the same slope, so the two output rings will need joining.
    fn shares_bottom(&self, horz: &TEdge, other: &TEdge) -> bool {
        other.curr.x == horz.bot.x
            && other.curr.y == horz.bot.y
            && other.wind_delta != 0
            && other.out_idx >= 0
            && other.curr.y > other.top.y
            && slopes_equal_edges(horz, other, self.use_full_range)
    }

    fn get_next_in_ael(&self, e: usize, direction: Direction) -> Option<usize> {
        if direction == Direction::LeftToRight {
            self.edges[e].next_in_ael
        } else {
            self.edges[e].prev_in_ael
        }
    }

    fn is_maxima(&self, e: usize, y: f64) -> bool {
        self.edges[e].top.y as f64 == y && self.edges[e].next_in_lml.is_none()
    }

    fn is_intermediate(&self, e: usize, y: f64) -> bool {
        self.edges[e].top.y as f64 == y && self.edges[e].next_in_lml.is_some()
    }

    /// C# `GetMaximaPair`.
    pub(super) fn get_maxima_pair(&self, e: usize) -> Option<usize> {
        let (next, prev) = (self.edges[e].next, self.edges[e].prev);
        if self.edges[next].top == self.edges[e].top && self.edges[next].next_in_lml.is_none() {
            Some(next)
        } else if self.edges[prev].top == self.edges[e].top
            && self.edges[prev].next_in_lml.is_none()
        {
            Some(prev)
        } else {
            None
        }
    }

    /// C# `GetMaximaPairEx`: as above but returns null if MaxPair isn't in AEL (unless it's
    /// horizontal).
    fn get_maxima_pair_ex(&self, e: usize) -> Option<usize> {
        let result = self.get_maxima_pair(e)?;
        let r = &self.edges[result];
        if r.out_idx == SKIP || (r.next_in_ael == r.prev_in_ael && !is_horizontal(r)) {
            return None;
        }
        Some(result)
    }

    /// C# `ProcessEdgesAtTopOfScanbeam`.
    pub(super) fn process_edges_at_top_of_scanbeam(
        &mut self,
        top_y: i64,
    ) -> Result<(), ClipperError> {
        let mut e = self.active_edges;
        while let Some(mut edge) = e {
            //1. process maxima, treating them as if they're 'bent' horizontal edges,
            //   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
            let mut is_maxima_edge = self.is_maxima(edge, top_y as f64);

            if is_maxima_edge {
                let e_max_pair = self.get_maxima_pair_ex(edge);
                is_maxima_edge = match e_max_pair {
                    None => true,
                    Some(pair) => !is_horizontal(&self.edges[pair]),
                };
            }

            if is_maxima_edge {
                if self.strictly_simple {
                    self.insert_maxima(self.edges[edge].top.x);
                }
                let e_prev = self.edges[edge].prev_in_ael;
                self.do_maxima(edge)?;
                e = match e_prev {
                    None => self.active_edges,
                    Some(prev) => self.edges[prev].next_in_ael,
                };
            } else {
                //2. promote horizontal edges, otherwise update Curr.X and Curr.Y ...
                let promote = self.is_intermediate(edge, top_y as f64)
                    && self.edges[edge]
                        .next_in_lml
                        .is_some_and(|next| is_horizontal(&self.edges[next]));
                if promote {
                    edge = self.update_edge_into_ael(edge)?;
                    if self.edges[edge].out_idx >= 0 {
                        self.add_out_pt(edge, self.edges[edge].bot);
                    }
                    self.add_edge_to_sel(edge);
                } else {
                    self.edges[edge].curr.x = top_x(&self.edges[edge], top_y);
                    self.edges[edge].curr.y = top_y;
                }
                //When StrictlySimple and 'e' is being touched by another edge, then
                //make sure both edges have a vertex here ...
                if self.strictly_simple {
                    if let Some(prev) = self.edges[edge].prev_in_ael {
                        let (cur, p) = (&self.edges[edge], &self.edges[prev]);
                        if cur.out_idx >= 0
                            && cur.wind_delta != 0
                            && p.out_idx >= 0
                            && p.curr.x == cur.curr.x
                            && p.wind_delta != 0
                        {
                            let ip = cur.curr;
                            let op = self.add_out_pt(prev, ip);
                            let op2 = self.add_out_pt(edge, ip);
                            self.add_join(op, op2, ip); //StrictlySimple (type-3) join
                        }
                    }
                }

                e = self.edges[edge].next_in_ael;
            }
        }

        //3. Process horizontals at the Top of the scanbeam ...
        self.process_horizontals()?;
        self.maxima_list = None;
        self.maxima.clear();

        //4. Promote intermediate vertices ...
        let mut e = self.active_edges;
        while let Some(mut edge) = e {
            if self.is_intermediate(edge, top_y as f64) {
                let mut op = None;
                if self.edges[edge].out_idx >= 0 {
                    op = Some(self.add_out_pt(edge, self.edges[edge].top));
                }
                edge = self.update_edge_into_ael(edge)?;

                //if output polygons share an edge, they'll need joining later ...
                let e_prev = self.edges[edge].prev_in_ael;
                let e_next = self.edges[edge].next_in_ael;
                if let Some(op) = op {
                    let cur = self.edges[edge].clone();
                    if let Some(prev) =
                        e_prev.filter(|&p| self.continues_beside(&cur, &self.edges[p]))
                    {
                        let op2 = self.add_out_pt(prev, cur.bot);
                        self.add_join(op, op2, cur.top);
                    } else if let Some(next) =
                        e_next.filter(|&n| self.continues_beside(&cur, &self.edges[n]))
                    {
                        let op2 = self.add_out_pt(next, cur.bot);
                        self.add_join(op, op2, cur.top);
                    }
                }
            }
            e = self.edges[edge].next_in_ael;
        }
        Ok(())
    }

    /// Step 4's test of a neighbour of a just-promoted edge: a contributing closed edge at the
    /// promoted edge's bottom with the same slope (C#'s inline conditions, `op != null` aside).
    fn continues_beside(&self, cur: &TEdge, other: &TEdge) -> bool {
        other.curr.x == cur.bot.x
            && other.curr.y == cur.bot.y
            && other.out_idx >= 0
            && other.curr.y > other.top.y
            && slopes_equal4(cur.curr, cur.top, other.curr, other.top, self.use_full_range)
            && cur.wind_delta != 0
            && other.wind_delta != 0
    }

    fn do_maxima(&mut self, e: usize) -> Result<(), ClipperError> {
        let Some(e_max_pair) = self.get_maxima_pair_ex(e) else {
            if self.edges[e].out_idx >= 0 {
                self.add_out_pt(e, self.edges[e].top);
            }
            self.delete_from_ael(e);
            return Ok(());
        };

        let mut e_next = self.edges[e].next_in_ael;
        while let Some(next) = e_next {
            if next == e_max_pair {
                break;
            }
            let top = self.edges[e].top;
            self.intersect_edges(e, next, top);
            self.swap_positions_in_ael(e, next);
            e_next = self.edges[e].next_in_ael;
        }

        if self.edges[e].out_idx == UNASSIGNED && self.edges[e_max_pair].out_idx == UNASSIGNED {
            self.delete_from_ael(e);
            self.delete_from_ael(e_max_pair);
        } else if self.edges[e].out_idx >= 0 && self.edges[e_max_pair].out_idx >= 0 {
            let top = self.edges[e].top;
            self.add_local_max_poly(e, e_max_pair, top);
            self.delete_from_ael(e);
            self.delete_from_ael(e_max_pair);
        } else if self.edges[e].wind_delta == 0 {
            let top = self.edges[e].top;
            if self.edges[e].out_idx >= 0 {
                self.add_out_pt(e, top);
                self.edges[e].out_idx = UNASSIGNED;
            }
            self.delete_from_ael(e);

            if self.edges[e_max_pair].out_idx >= 0 {
                self.add_out_pt(e_max_pair, top);
                self.edges[e_max_pair].out_idx = UNASSIGNED;
            }
            self.delete_from_ael(e_max_pair);
        } else {
            return Err(ClipperError::DoMaximaError);
        }
        Ok(())
    }
}
